#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "gl/glew.h"
#include <gl/gl.h>
#include <cmath>
#include <ctime>

#include "vec.h"
#include "graphics.h"
#include "ui.h"
#include "ui_3d_centerview.h"
#include "fps.h"
#include "timer.h"
#include "profiler.h"
#include "space.h"

using namespace std;
using namespace ui;

class space_panel : public ui_3d_centerview {
private:
    typedef vec<double, 3> vec3;

    frame & rep;
    timer forward_timer;
    fps fpscounter;
    space<double, 3> sp;
    bool center, pause;
    u32 center_index;

    static double constexpr forward_time = .01, forward_mdps = .01, forward_G = .001;
public:
    space_panel(frame & rep) : rep(rep), forward_timer(10, [this] (DWORD t) -> UINT {
        if (!pause) sp.forward(forward_time, forward_mdps, forward_G);
        this->rep.refresh();
        return 10;
    }), center(false), pause(true), center_index(0) {
        autoRepaint(false);
        srand(clock());
        double constexpr rm = RAND_MAX;
        for (u32 i = 0; i < 10; ++i) {
            sp.add_particle(particle<double, 3>(
                    vec3(rand() / rm - .5, rand() / rm - .5, rand() / rm - .5),
                    vec3(rand() / rm - .5, rand() / rm - .5, rand() / rm - .5) * .01,
                    rand() / rm * .05 + .05, rand() / rm * .05 + .05)
            );
        }

//        vec3 zero(0, 0, 0), o(-.25, -.25, -.25), x(.5, 0, 0), y(0, .5, 0), z(0, 0, .5);
//        double r = sqrt(3.) / 16. * .9;
//        sp.add_particle(particle<double, 3>(o, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + x, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + y, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + z, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + x + y, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + y + z, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + z + x, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + x + y + z, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (x + y) * .5, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (y + z) * .5, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (z + x) * .5, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (x + y) * .5 + z, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (y + z) * .5 + x, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (z + x) * .5 + y, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + (x + y + z) * .25, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + x * .25 + (y + z) * .75, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + y * .25 + (z + x) * .75, zero, r, 1));
//        sp.add_particle(particle<double, 3>(o + z * .25 + (x + y) * .75, zero, r, 1));
    }

    virtual void onKeyDown(int x, int y, std::vector<vk> const & keystroke) {
        if (keystroke.size() == 1) {
            vk key = keystroke[0];
            switch (key) {
            case vk::esc:
                center = false;
                break;
            case vk::left:
                center = true;
                if (center_index <= 0) center_index = sp.particles().size() - 1;
                else --center_index;
                break;
            case vk::right:
                center = true;
                if (center_index + 1 >= sp.particles().size()) center_index = 0;
                else ++center_index;
                break;
            case vk::space:
                pause = !pause;
                break;
            case vk::shift:
                if (pause) {
                    sp.forward(forward_time, forward_mdps, forward_G);
                    rep.refresh();
                }
                break;
            }
        }
    }

    virtual void paint(opengl & gl) {
        ui_3d_centerview::paint(gl);

        glPushAttrib(GL_ENABLE_BIT);
        f32 lightpos[4] = {0, 0, 0, 1},
            lightamb[3] = {1, 1, 1},
            lightdif[3] = {1, 1, 1},
            lightspe[3] = {1, 1, 1};
        glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
        glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightdif);
        glLightfv(GL_LIGHT0, GL_SPECULAR, lightspe);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        vector<particle<double, 3>> const & p = sp.particles();
        if (!p.empty()) {
            vec<double, 3> trans;
            if (center && center_index < p.size()) {
                trans = p[center_index].position();
            }
            glTranslatef(-trans[0], -trans[1], -trans[2]);
            for (auto i = p.begin(); i < p.end(); ++i) {
                vec<double, 3> const & p = i->position();
                float radius = i->radius();
                float density = i->mass() / (radius * radius * radius * (4. / 3. * pi));
                float color = 1 - exp(-density);
                glColor3f(color, color, color);
                gl.drawSphere(p[0], p[1], p[2], i->radius(), 4);
            }
            glPopMatrix();
        }
        glPopAttrib();

        gl.mode_2d();
        stringstream str;
        if (center && center_index < p.size()) {
            gl.pushMatrix();
            particle<double, 3> par = p[center_index];

            str << setprecision(3) << fixed << setfill(' ');
            {
                vec<double, 3> pos = par.position();
                str << "pos = (" << setw(6) << pos[0] << ", " << setw(6) << pos[1] << ", " << setw(6) << pos[2] << ")";
                gl.translate(getWidth(), 0, 0);
                gl.drawText(str.str(), 20, false, h_align::right, v_align::top);
                str.str("");
            }{
                vec<double, 3> vel = par.velocity();
                str << "vel = (" << setw(6) << vel[0] << ", " << setw(6) << vel[1] << ", " << setw(6) << vel[2] << ")";
                gl.translate(0, 20, 0);
                gl.drawText(str.str(), 20, false, h_align::right, v_align::top);
                str.str("");
            }{
                double mass = par.mass();
                str << "mass = " << setw(6) << mass;
                gl.translate(0, 20, 0);
                gl.drawText(str.str(), 20, false, h_align::right, v_align::top);
                str.str("");
            }{
                double radius = par.radius();
                str << "rad = " << setw(6) << radius;
                gl.translate(0, 20, 0);
                gl.drawText(str.str(), 20, false, h_align::right, v_align::top);
                str.str("");
            }
            str.copyfmt(std::ios(nullptr));

            gl.popMatrix();
        }
        str << (float) fpscounter;
        if (pause) str << " PAUSED";
        string str_fps = str.str();
        float w = 0;
        gl.getTextExtent(str_fps, 20, &w, nullptr, nullptr);
        gl.setColor(0, 255, 255);
        gl.fillRect(0, 0, w, 20);
        gl.setColor(255, 255, 255);
        gl.drawText(str.str(), 20, false, h_align::left, v_align::top);
        ++fpscounter;
    }
};

int main() {
    frame mainframe("Gravity");
    space_panel mainpanel(mainframe);
    mainframe.setContent(&mainpanel);
    mainframe.setVisible(true);

    frame::startMessageLoop();
    return 0;
}
