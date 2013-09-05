
from iocbuilder import Device, AutoSubstitution, Architecture, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, _ADBaseTemplate, simDetector

class _pcocam(AutoSubstitution):
    TemplateFile="pcocam.template"
    SubstitutionOverwrites = [_ADBaseTemplate]
    
class pcocam(_ADBase):
    """Create a PCO camera detector"""
    _SpecificTemplate = _pcocam
    def __init__(self, PCOWIDTH, PCOHEIGHT, BUFFERS=50, MEMORY=-1, **args):
        self.__super.__init__(**args)
        self.__dict__.update(locals())
        
    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PCOWIDTH = Simple('PCO sensor width', int),
        PCOHEIGHT  = Simple('PCO sensor height', int),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
            'plugin callbacks', int),
        MEMORY  = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int))
    LibFileList = []
    DbdFileList = []
    SysLibFileList = []
    MakefileStringList = []
    epics_host_arch = Architecture()
    # For any windows architecture, install the pcocam libraries
    # and configure the required linker flags
    if epics_host_arch.find('win') >= 0:
        LibFileList += ['pcocamSupport', 'SC2_DLG', 'SC2_Cam','PCO_CDLG','Pco_conv' ]
        SysLibFileList += ['nafxcw']
        DbdFileList += ['pcocamSupport']
        MakefileStringList += ['%(ioc_name)s_LDFLAGS_WIN32 += /NOD:nafxcwd.lib /NOD:nafxcw.lib']
    
    def Initialise(self):
        print '# pcocam_config(   portName, maxSizeX, maxSizeY, maxBuffers, maxMemory )'
        print '  pcocam_config( %(PORT)10s, %(PCOWIDTH)8s, %(PCOHEIGHT)8s, %(BUFFERS)10d, %(MEMORY)9d )' % self.__dict__

def pcocam_sim(PCOWIDTH, PCOHEIGHT, **kwargs):
    return simDetector( WIDTH=PCOWIDTH, HEIGHT=PCOHEIGHT, **kwargs)

SetSimulation(pcocam, pcocam_sim)
