ADWinPco
=====

EPICS areaDetector driver for the PCO cameras

Credits and licensing
---------------------

Original development of source code in this module from Diamond Light Source. Released under 
the Apache V2 license. See LICENSE.

The camera drivers are provided by PCO and redistributed here with permission from the vendor 
in binary, unmodified form. The drivers are downloaded from the 
[PCO support website](https://www.pco.de/support/interface/scmos-cameras/)


Supported Cameras
-----------------

This driver has been tested with the following combination of cameras and framegrabbers:

| Camera model  | Framegrabber   | Interface     | ADPcoWin Release |
|---------------|----------------|---------------|------------------|
| pco.4000      | microEnable IV | CameraLink    | 3-3-1            |
| pco.1600      | microEnable IV | CameraLink    | 3-3-1            |
| pco.dimax     | microEnable IV | CameraLink    | 3-3-1            |
| pco.edge CL   | microEnable IV | CameraLink    | 3-3-1*           |
| pco.edge CLHS | microEnable V  | CameraLink HS | 4-1^             |

*There are reported problems with 3-3-1 with pco.edge (see Known Issues)
^Version 4 has only been built and tested with the CLHS .dll 

Operating Systems
-----------------

Current drivers work with 64 bit Windows 7, Server 2008 and Server 2012.

Known Issues
------------

* In releases 3-2, 3-3, 3-3-1, the pco.edge may disarm while in the middle of an externally triggered acquisition. It is recommended to use release 3-1 for stability.
* Release 4-0 currently only supports PCO Edge 5.5 CLHS. There is testing planned for supporting CL cameras.
