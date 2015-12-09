
from iocbuilder import Device, AutoSubstitution, Architecture
from iocbuilder.arginfo import *

from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, includesTemplates, makeTemplateInstance
from iocbuilder.modules.asyn import AsynPort

@includesTemplates(ADBaseTemplate)

class _pcocam2(AutoSubstitution):
    TemplateFile="pco.template"

class pcocam2(AsynPort):
    """Create a PCO camera detector"""
    Dependencies = (ADCore,)
    _SpecificTemplate = _pcocam2
    UniqueName = "PORT"

    def __init__(self, PORT, BUFFERS=50, MEMORY=-1, **args):
        self.__super.__init__(PORT)
        self.__dict__.update(locals())
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    # __init__ arguments
    ArgInfo = _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PORT = Simple('Port name for the detector', str),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created', int),
        MEMORY  = Simple('Max memory to allocate', int))
    LibFileList = ['pcocam2']
    DbdFileList = ['pcocam2Support']
    SysLibFileList = []
    MakefileStringList = []
    epics_host_arch = Architecture()
    # For any windows architecture, install the pcocam libraries
    # and configure the required linker flags
    if epics_host_arch.find('win') >= 0:
        LibFileList += ['SC2_DLG', 'SC2_Cam','PCO_CDLG','Pco_conv' ]
        SysLibFileList += ['windowscodecs', 'Comdlg32', 'Winspool', 'Comctl32', 'nafxcw']
        DbdFileList += ['pcocam2HardwareSupport']
        if epics_host_arch.find('debug') >= 0:
            MakefileStringList += ['%(ioc_name)s_LDFLAGS_WIN32 += /NOD:nafxcwd.lib /NOD:nafxcw.lib /NOD:libcmt']
        else:
            MakefileStringList += ['%(ioc_name)s_LDFLAGS_WIN32 += /NOD:nafxcwd.lib /NOD:nafxcw.lib']

    def Initialise(self):
        print 'pcoConfig("%(PORT)s", %(BUFFERS)d, %(MEMORY)d)' % self.__dict__
        if self.epics_host_arch.find('win') >= 0:
            print 'pcoApiConfig("%(PORT)s")' % self.__dict__
        else:
            print 'simulationApiConfig("%(PORT)s")' % self.__dict__

