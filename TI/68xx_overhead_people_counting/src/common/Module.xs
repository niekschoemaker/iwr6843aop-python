/******************************************************************************
 * FILE PURPOSE: INCLUDE Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the module specification for the package documentation.
 *
 * Copyright (C) 2016, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule (java.lang.System.getenv("MMWAVE_SDK_INSTALL_PATH") + "/scripts/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the internal module specific include
 *  files to the release package
 **************************************************************************/
function modBuild()
{
    /* Add all the .h files to the release package. */
    var incFiles = libUtility.listAllFiles (".h", "common", false);
    for (var k = 0 ; k < incFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = incFiles[k];

    /* Add all the .c files to the release package. */
    var srcFiles = libUtility.listAllFiles (".c", "common", false);
    for (var k = 0 ; k < srcFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = srcFiles[k];

    /* Add all the .cfg files to the release package. */
    var cfgFiles = libUtility.listAllFiles (".cfg", "common", false);
    for (var k = 0 ; k < cfgFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = cfgFiles[k];

    /* Add all the .mak files to the release package. */
    var makFiles = libUtility.listAllFiles (".mak", "common", false);
    for (var k = 0 ; k < makFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = makFiles[k];

    /* Add all the .cmd files to the release package. */
    var cmdFiles = libUtility.listAllFiles (".cmd", "common", false);
    for (var k = 0 ; k < cmdFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = cmdFiles[k];

    /* Add all the .txt files to the release package. */
    var cmdFiles = libUtility.listAllFiles (".txt", "common", false);
    for (var k = 0 ; k < cmdFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = cmdFiles[k];

}

