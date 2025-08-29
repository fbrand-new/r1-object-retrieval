/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "checkObjectGrasped.h"

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yCError(CHECK_OBJECT_GRASPED,"YARP server not available!");
        return 1;
    }

    CheckObjectGrasped module;
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("checkObjectGrasped.ini");
    rf.setDefaultContext("checkObjectGrasped");
    rf.configure(argc, argv);

    return module.runModule(rf);
}
