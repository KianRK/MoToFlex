/*
    Main file for the Python interface.
*/

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "nao/nao.h"
#include "simulation/world.h"
#include <iostream>
#include <unistd.h>
#include "adapter/anglefrompython.h"
#include "tools/kinematics.h"

// https://stackoverflow.com/questions/2610421/how-to-create-make-so-files-from-code-written-in-c-or-c-that-are-usable-from
// https://docs.python.org/3/extending/
// https://www.digitalocean.com/community/tutorials/calling-c-functions-from-python
// /Users/civic/Fraunhofer-ownCloud/Archiv/db/db-schenker-videoanalytics/Tracker/WeightPairs

char mod_docs[] = "Interface to MoToFlex WalkingSimulator.";
AngleFromPython angleAdapters[NUM_OF_NAOS];

// global angles array as this is retrieved from python once,
// written into this array, pointer to this array
// used in AngleFromPython and then copied from this array
// into target array as target array may be modified later
float latest_angles[NUM_OF_JOINTS];
float pose[NUM_OF_JOINTS];

PyObject *floatArrayToPy(float array[], int length)
{
    PyObject *pylist, *item;
    pylist = PyList_New(length);
    for (int i = 0; i < length; i++)
    {
        item = PyFloat_FromDouble((double)array[i]);
        PyList_SetItem(pylist, i, item);
    }
    return pylist;
}

PyObject *doubleArrayToPy(double array[], int length)
{
    float *fa = new float[length];
    for (int i = 0; i < length; i++)
        fa[i] = array[i];
    return floatArrayToPy(fa, length);
}

bool parseFloatArray(PyObject* args, float* floats, int size)
{
    PyObject* inputList;
    if (!PyArg_ParseTuple(args, "O", &inputList))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return false;
    }
    
    if (!PyList_Check(inputList) || PyList_Size(inputList) != size)
    {
        // Das Eingabeobjekt ist keine Liste oder hat nicht die erwartete Größe
        printf("Input object is no list and doesn't have the expected size.\n");
        return false;
    }
    
    for (int i = 0; i < size; i++)
    {
        PyObject* item = PyList_GetItem(inputList, i);
        if (!PyFloat_Check(item))
        {
            // Das Element in der Liste ist kein Float
            printf("Element is no float.\n");
            return false;
        }
        
        floats[i] = PyFloat_AsDouble(item);
    }
    
    return true;
}

static PyObject *step(PyObject *self, PyObject *args)
{   
    if (!parseFloatArray(args, latest_angles, NUM_OF_JOINTS))
    {
        // Fehler beim Parsen des Arrays
        printf("Error parsing array.\n");
        return NULL;
    }

    for (int i=0; i<NUM_OF_NAOS; i++)
	{
        angleAdapters[i].setAngles(latest_angles, NUM_OF_JOINTS);
    }

    int framesBetweenAngles=((int)(SOURCE_DT/parms.frameLen)+1);

    for (int i = 0; i < framesBetweenAngles; i++)
        simLoop(false);
    
    return PyLong_FromLong(1.L);
}

static PyObject *inverseKinematics(PyObject *self, PyObject *args)
{   // Independet from the NUM_OF_JOINTS, here we always work with the
    // full DOF of the Nao legs.

    float angles[12];
    float pose[12];

    if (!parseFloatArray(args, pose, 12))
    {
        // Fehler beim Parsen des Arrays
        printf("Error parsing array.\n");
        return NULL;
    }

    inverseKinematics(true, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], angles);
    t0InverseKinematics(false, pose[6], pose[7], pose[8], pose[9], pose[10], angles[0], &angles[6]);
    return floatArrayToPy(angles, sizeof(angles) / sizeof(float));
}


static PyObject *forwardKinematics(PyObject *self, PyObject *args)
{   // Independet from the NUM_OF_JOINTS, here we always work with the
    // full DOF of the Nao legs.

    float angles[12];
    double pose[12];

    if (!parseFloatArray(args, angles, 12))
    {
        // Fehler beim Parsen des Arrays
        printf("Error parsing array.\n");
        return NULL;
    }

    forwardKinematics(true, angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], pose);
    forwardKinematics(false, angles[6], angles[7], angles[8], angles[9], angles[10], angles[11], &pose[6]);
    return doubleArrayToPy(pose, sizeof(pose) / sizeof(double));
}

static PyObject *reset(PyObject *self, PyObject *args)
{
    quiet = true;
    
    const char *configPath;

    if (!PyArg_ParseTuple(args, "s", &configPath))
        return NULL;

    quitSimulation();
    if (!initSimulation(configPath, true))
    {
        printf("Error loading %s", configPath);
        PyErr_SetString(PyExc_Exception, "Init error, maybe wrong config path?");
        return NULL;
    }
    for (int i=0; i<NUM_OF_NAOS; i++)
	{
        latest_angles[i] = 0.f;
        angleAdapters[i].setAngles(latest_angles, NUM_OF_JOINTS);
    }
    return PyLong_FromLong(0.L);
}

static PyObject *getBox6DPose(PyObject *self, PyObject *args)
{
    int bodyID;
    dReal p[6];

    if (!PyArg_ParseTuple(args, "i", &bodyID))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    worlds[Nao::body].nao.getPosition(bodyID, p);
    worlds[Nao::body].nao.getOrientation(bodyID, &p[3]);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *getBodyOrientationQuaternion(PyObject *self, PyObject *args)
{
    dReal quat[4];

    worlds[Nao::body].nao.getBodyOrientationQuaternion(quat);

    return floatArrayToPy(quat, sizeof(quat) / sizeof(dReal));
}

static PyObject *getJointAngles(PyObject *self, PyObject *args)
{
    float angles[NUM_OF_JOINTS];

    worlds[Nao::body].nao.getAngles(angles);

    return floatArrayToPy(angles, NUM_OF_JOINTS);
}

static PyObject *getJointVelocities(PyObject *self, PyObject *args)
{
    float velocities[NUM_OF_JOINTS];

    worlds[Nao::body].nao.getJointVelocities(velocities);

    return floatArrayToPy(velocities, NUM_OF_JOINTS);
}

static PyObject *getVelocity(PyObject *self, PyObject *args)
{
    dReal p[3];

    worlds[Nao::body].nao.getVelocity(p);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *getAngularVelocity(PyObject *self,PyObject *args)
{
    dReal p[3];

    worlds[Nao::body].nao.getAngularVelocity(p);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *getLeftFootVelocity(PyObject *self, PyObject *args)
{
    dReal p[3];

    worlds[Nao::footLeft].nao.getVelocity(p);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *getRightFootVelocity(PyObject *self, PyObject *args)
{
    dReal p[3];

    worlds[Nao::rightLeft].nao.getVelocity(p);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *get6DPose(PyObject *self, PyObject *args)
{
    dReal p[6];

    worlds[Nao::body].nao.getPosition(p);
    worlds[Nao::body].nao.getOrientation(&p[3]);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *getBoxSize(PyObject *self, PyObject *args)
{
    int bodyID;
    dReal p[3];

    if (!PyArg_ParseTuple(args, "i", &bodyID))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    worlds[Nao::body].nao.getSize(bodyID, p);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *addForce(PyObject *self, PyObject *args)
{
    int bodyID;
    dReal fx, fy, fz;

    if (!PyArg_ParseTuple(args, "ifff", &bodyID, &fx, &fy, &fz))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    worlds[Nao::body].nao.addForce(bodyID, fx, fy, fz);

    return PyLong_FromLong(0.L);
}

static PyObject *getCoM(PyObject *self, PyObject *args)
{
    float fap[3];
    Vector3<double> p = worlds[Nao::body].nao.getCoM();

    for (int i = 0; i < 3; i++)
        fap[i] = p[i];

    return floatArrayToPy(fap, sizeof(fap) / sizeof(float));
}

static PyObject *getBoxSizeInFlexbox(PyObject *self, PyObject *args)
{
    int bodyID;
    int flexboxID;
    dReal *p;

    if (!PyArg_ParseTuple(args, "ii", &flexboxID, &bodyID))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    p = worlds[Nao::body].nao.getFlexbox(flexboxID)->getBox(bodyID)->getSize();

    return floatArrayToPy(p, 3);
}

static PyObject *getBoxPoseInFlexbox(PyObject *self, PyObject *args)
{
    int bodyID;
    int flexboxID;
    dReal p[6];

    if (!PyArg_ParseTuple(args, "ii", &flexboxID, &bodyID))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    worlds[Nao::body].nao.getFlexbox(flexboxID)->getBox(bodyID)->getPosition(p);
    worlds[Nao::body].nao.getFlexbox(flexboxID)->getBox(bodyID)->getOrientation(&p[3]);

    return floatArrayToPy(p, sizeof(p) / sizeof(dReal));
}

static PyObject *getBodyCount(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(Nao::numOfBoxes);
}

static PyObject *getFlexBoxCount(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(Nao::numOfFlexBoxes);
}

static PyObject *getBoxCountInFlexBox(PyObject *self, PyObject *args)
{
    int flexboxID;

    if (!PyArg_ParseTuple(args, "i", &flexboxID))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    return PyLong_FromLong(worlds[Nao::body].nao.getFlexbox(flexboxID)->getNumOfBoxes());
}


static PyObject *footContact(PyObject *self, PyObject *args)
{
    int footNum;

    if (!PyArg_ParseTuple(args, "i", &footNum))
    {
        // Fehler beim Parsen des Eingabeobjekts
        printf("Error parsing input object.\n");
        return NULL;
    }

    return PyBool_FromLong(worlds[Nao::body].hasContact(footNum));
}

static PyObject *init(PyObject *self, PyObject *args)
{
    quiet = true;
    for (int i=0; i<NUM_OF_NAOS; i++)
	{
        behaviour[i] = &angleAdapters[i];
    }
    
    return PyLong_FromLong(0.L);
}

static PyObject *isRunning(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(running);
}

static PyObject *test(PyObject *self, PyObject *args)
{
    return PyFloat_FromDouble(1.4);
}

static PyMethodDef WalkingSimulatorMethods[] = {

    {"step", step, METH_VARARGS, "Make simulation step."},
    {"reset", reset, METH_VARARGS, "Reset the simulation."},
    {"init", init, METH_VARARGS, "Initialize the simulation."},
    {"test", test, METH_VARARGS, "Test function."},
    {"foot_contact", footContact, METH_VARARGS, "Has the foot contact to ground?"},
    {"get_com_pos", getCoM, METH_VARARGS, "Get position of CoM of robot."},
    {"get_body_orientation_quaternion", getBodyOrientationQuaternion, METH_VARARGS, "Get orientation of body as quaternion"}
    {"get_joint_angles", getJointAngles, METH_VARARGS, "Get the measured angles of legs."},
    {"get_joint_velocities", getJointVelocities, METH_VARARGS, "Get the measured velocities of joints"},
    {"get_velocity", getVelocity, METH_VARARGS, "Get velocity of robot, which is velocity of upper body."},
    {"get_angular_velocity", getAngularVelocity, METH_VARARGS, "Get angular velocity of robots upper body."},
    {"get_left_foot_velocity", getLeftFootVelocity, METH_VARARGS, "Get velocity of left foot of the robot."},
    {"get_right_foot_velocity", getRightFootVelocity, METH_VARARGS, "Get velocity of right foot of the robot."},
    {"get_right_foot_force", getRightFootForce, METH_VARARGS, "Get force of right foot of the robot."},
    {"get_left_foot_force", getLeftFootForce, METH_VARARGS, "Get force of left foot of the robot."},
    {"get_6d_pose", get6DPose, METH_VARARGS, "Get 6D pose of upper body."},
    {"get_box_6d_pose", getBox6DPose, METH_VARARGS, "Get 6D pose of body."},
    {"get_box_size", getBoxSize, METH_VARARGS, "Get size of body."},
    {"get_box_count", getBodyCount, METH_VARARGS, "Get number of boxes for robot."},
    {"get_flexbox_count", getFlexBoxCount, METH_VARARGS, "Get number of flexible boxes for robot."},
    {"get_box_count_in_flexbox", getBoxCountInFlexBox, METH_VARARGS, "Get number of boxex in flexible box."},
    {"get_box_size_in_flexbox", getBoxSizeInFlexbox, METH_VARARGS, "Get size of box in flexbox."},
    {"get_box_pose_in_flexbox", getBoxPoseInFlexbox, METH_VARARGS, "Get pose of box in flexbox."},
    {"is_running", isRunning, METH_VARARGS, "Is the simulation still running?"},
    {"inverse_kinematics", inverseKinematics, METH_VARARGS, "Inverse kinematics for Nao."},
    {"forward_kinematics", forwardKinematics, METH_VARARGS, "Forward kinematics for Nao."},
    {"add_force", addForce, METH_VARARGS, "Adds the given force to the body."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef spammodule = {
    PyModuleDef_HEAD_INIT,
    "WalkingSimulator",   /* name of module */
    mod_docs, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
               or -1 if the module keeps state in global variables. */
    WalkingSimulatorMethods
};

PyMODINIT_FUNC
PyInit_WalkingSimulator(void)
{
    return PyModule_Create(&spammodule);
}