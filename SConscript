
from building import *

cwd     = GetCurrentDir()
src     = Glob('*.c') + Glob('*.cpp')
path    = [cwd]

group = DefineGroup('sr04', src, depend = ['PKG_USING_SR04'], CPPPATH = path)

Return('group')
