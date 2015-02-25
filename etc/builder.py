
from iocbuilder import Device, AutoSubstitution, Architecture
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, _ADBaseTemplate, simDetector

class _pcocam2(AutoSubstitution):
    TemplateFile="pco.template"
    SubstitutionOverwrites = [_ADBaseTemplate]

class pcocam2(_ADBase):
    """Create a PCO camera detector"""
    _SpecificTemplate = _pcocam2
    def __init__(self, BUFFERS=50, MEMORY=-1, **args):
        self.__super.__init__(**args)
        self.__dict__.update(locals())

    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
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
        MakefileStringList += ['%(ioc_name)s_LDFLAGS_WIN32 += /NOD:nafxcwd.lib /NOD:nafxcw.lib']

    def Initialise(self):
        print 'pcoConfig("%(PORT)s", %(BUFFERS)d, %(MEMORY)d)' % self.__dict__
        if self.epics_host_arch.find('win') >= 0:
            print 'pcoApiConfig("%(PORT)s")' % self.__dict__
        else:
            print 'simulationApiConfig("%(PORT)s")' % self.__dict__

