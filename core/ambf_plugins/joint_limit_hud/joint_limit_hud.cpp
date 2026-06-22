//==============================================================================
//  On-screen joint-limit HUD indicator (AMBF simulator plugin)
//
//  Renders a small text panel in the simulation window listing every limited
//  revolute/prismatic joint and how close it is to its nearest limit:
//      green  = comfortable
//      yellow = approaching a limit (last ~10% of travel)
//      red    = at a limit (saturated)
//
//  Purpose: during teleop it is otherwise hard to tell whether the robot fails
//  to reach a pose because a joint is pinned at its limit (workspace/feasibility)
//  or because of an IK/tracking problem. A red joint line means "limit reached";
//  if nothing is red but the end-effector still isn't where commanded, it's a
//  tracking/IK issue (by elimination).
//
//  Detection is purely geometric (joint position vs URDF limits) and works for
//  any loaded robot. No ROS, no mesh recoloring, no joint-limit changes.
//==============================================================================

#include <afFramework.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace std;
using namespace ambf;

class afJointLimitHudPlugin: public afSimulatorPlugin{
public:
    virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override {}
    virtual void reset() override {}
    virtual bool close() override;

private:
    struct Tracked{
        afJointPtr joint = nullptr;
        double lower = 0.0;
        double upper = 0.0;
        double range = 0.0;
        bool prismatic = false;
        cLabel* label = nullptr;
    };

    void buildLabels();

    afWorldPtr m_world = nullptr;
    afCameraPtr m_cam = nullptr;
    vector<Tracked> m_tracked;
    cLabel* m_header = nullptr;
    bool m_built = false;

    // Saturation thresholds (radians for revolute, meters for prismatic).
    const double m_epsRev = 0.035;   // ~2 deg
    const double m_epsPris = 0.002;  // 2 mm
};


int afJointLimitHudPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    m_world = a_afWorld;
    if (m_world == nullptr){
        cerr << "ERROR! JOINT_LIMIT_HUD: NULL world handle, plugin disabled" << endl;
        return -1;
    }

    // Collect every limited revolute/prismatic joint in the world. The child
    // body is irrelevant for a text-only HUD, so we just keep the joints.
    afJointVec joints = m_world->getJoints();
    for (size_t i = 0 ; i < joints.size() ; i++){
        afJointPtr j = joints[i];
        if (j == nullptr){
            continue;
        }
        if (j->m_jointType != afJointType::REVOLUTE && j->m_jointType != afJointType::PRISMATIC){
            continue;
        }
        double lo = j->getLowerLimit();
        double hi = j->getUpperLimit();
        double range = hi - lo;
        // Skip continuous / limitless joints (no usable range).
        if (!std::isfinite(range) || range < 1e-4){
            continue;
        }
        Tracked t;
        t.joint = j;
        t.lower = lo;
        t.upper = hi;
        t.range = range;
        t.prismatic = (j->m_jointType == afJointType::PRISMATIC);
        m_tracked.push_back(t);
    }

    // Grab the first camera; its front layer hosts the 2D HUD labels.
    afCameraVec cameras = m_world->getCameras();
    if (!cameras.empty()){
        m_cam = cameras[0];
    }

    cerr << "INFO! JOINT_LIMIT_HUD: tracking " << m_tracked.size()
         << " limited joint(s)" << (m_cam ? "" : " (no camera found - HUD disabled)") << endl;

    return 1;
}


void afJointLimitHudPlugin::buildLabels(){
    if (m_cam == nullptr){
        return;
    }
    cWorld* frontLayer = m_cam->getFrontLayer();
    if (frontLayer == nullptr){
        return;
    }

    cFontPtr font = NEW_CFONTCALIBRI20();

    m_header = new cLabel(font);
    m_header->setFontScale(0.8);
    m_header->m_fontColor.setWhite();
    frontLayer->addChild(m_header);

    for (size_t i = 0 ; i < m_tracked.size() ; i++){
        cLabel* l = new cLabel(font);
        l->setFontScale(0.8);
        l->m_fontColor.set(0.7f, 0.7f, 0.7f);
        frontLayer->addChild(l);
        m_tracked[i].label = l;
    }

    m_built = true;
}


void afJointLimitHudPlugin::graphicsUpdate(){
    if (m_cam == nullptr){
        return;
    }
    // Defer label/scene-graph creation to the render thread (first frame).
    if (!m_built){
        buildLabels();
        if (!m_built){
            return;
        }
    }

    const double rad2deg = 180.0 / 3.14159265358979323846;
    const int lineH = 18;
    const int x = 10;
    const int top = (int)m_cam->m_height - 20;

    int numAtLimit = 0;

    for (size_t i = 0 ; i < m_tracked.size() ; i++){
        Tracked& t = m_tracked[i];
        if (t.label == nullptr){
            continue;
        }

        double pos = t.joint->getPosition();
        if (pos < t.lower) pos = t.lower;
        if (pos > t.upper) pos = t.upper;

        double margin = std::min(pos - t.lower, t.upper - pos);
        double eps = t.prismatic ? m_epsPris : m_epsRev;
        double band = std::max(0.10 * (t.range * 0.5), 3.0 * eps);

        const char* state;
        if (margin <= eps){
            t.label->m_fontColor.set(1.0f, 0.15f, 0.15f);   // red
            state = "AT LIMIT";
            numAtLimit++;
        }
        else if (margin <= band){
            t.label->m_fontColor.set(1.0f, 0.78f, 0.0f);    // yellow/amber
            state = "near";
        }
        else{
            t.label->m_fontColor.set(0.1f, 0.85f, 0.1f);    // green
            state = "ok";
        }

        char buf[160];
        if (t.prismatic){
            snprintf(buf, sizeof(buf), "%-10s %+7.3f m  [%+.3f, %+.3f]  %s",
                     t.joint->getName().c_str(), pos, t.lower, t.upper, state);
        }
        else{
            snprintf(buf, sizeof(buf), "%-10s %+7.1f deg  [%+.1f, %+.1f]  %s",
                     t.joint->getName().c_str(), pos * rad2deg,
                     t.lower * rad2deg, t.upper * rad2deg, state);
        }
        t.label->setText(buf);
        t.label->setLocalPos(x, top - (int)(i + 1) * lineH);
    }

    if (m_header != nullptr){
        char hbuf[64];
        snprintf(hbuf, sizeof(hbuf), "JOINT LIMITS  (%d at limit)", numAtLimit);
        m_header->setText(hbuf);
        if (numAtLimit > 0){
            m_header->m_fontColor.set(1.0f, 0.15f, 0.15f);
        }
        else{
            m_header->m_fontColor.setWhite();
        }
        m_header->setLocalPos(x, top);
    }
}


bool afJointLimitHudPlugin::close(){
    if (m_cam != nullptr){
        cWorld* frontLayer = m_cam->getFrontLayer();
        if (frontLayer != nullptr){
            if (m_header != nullptr){
                frontLayer->removeChild(m_header);
            }
            for (size_t i = 0 ; i < m_tracked.size() ; i++){
                if (m_tracked[i].label != nullptr){
                    frontLayer->removeChild(m_tracked[i].label);
                }
            }
        }
    }
    if (m_header != nullptr){
        delete m_header;
        m_header = nullptr;
    }
    for (size_t i = 0 ; i < m_tracked.size() ; i++){
        if (m_tracked[i].label != nullptr){
            delete m_tracked[i].label;
            m_tracked[i].label = nullptr;
        }
    }
    return true;
}


AF_REGISTER_SIMULATOR_PLUGIN(afJointLimitHudPlugin)
