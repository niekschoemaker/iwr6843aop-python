/******************************************************************************
 * FILE PURPOSE: Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the "src" module specification for the package
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule (java.lang.System.getenv("MMWAVE_SDK_INSTALL_PATH") + "/scripts/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the package and add all the source
 *  files and header files to the release package which are required
 *  to build it.
 **************************************************************************/
function modBuild()
{
    /* Add all the .h files to the release package. */
    var incFiles = libUtility.listAllFiles (".h", "include");
    for (var k = 0 ; k < incFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = incFiles[k];
}

