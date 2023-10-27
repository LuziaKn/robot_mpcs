#jackal_n3_01_H10_noSlack : A fast customized optimization solver.
#
#Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
#
#
#This program is distributed in the hope that it will be useful.
#EMBOTECH makes NO WARRANTIES with respect to the use of the software 
#without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
#PARTICULAR PURPOSE. 
#
#EMBOTECH shall not have any liability for any damage arising from the use
#of the software.
#
#This Agreement shall exclusively be governed by and interpreted in 
#accordance with the laws of Switzerland, excluding its principles
#of conflict of laws. The Courts of Zurich-City shall have exclusive 
#jurisdiction in case of any dispute.
#
from distutils.ccompiler import new_compiler
from distutils import unixccompiler
c = new_compiler()
#from numpy.distutils.intelccompiler import IntelCCompiler
#c = IntelCCompiler()


import os
import sys
import distutils

# determine source file
sourcefile = os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","src","jackal_n3_01_H10_noSlack"+".c")

# determine lib file
if sys.platform.startswith('win'):
	libfile = os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","lib","jackal_n3_01_H10_noSlack"+".lib")
else:
	libfile = os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","lib","jackal_n3_01_H10_noSlack"+".so")	

# create lib dir if it does not exist yet
if not os.path.exists(os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","lib")):
	os.makedirs(os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","lib"))
								

				
# compile into object file
objdir = os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","obj")
if isinstance(c,unixccompiler.UnixCCompiler):
	#objects = c.compile([sourcefile], output_dir=objdir, extra_preargs=['-O3','-fPIC','-fopenmp','-mavx'])
	objects = c.compile([sourcefile], output_dir=objdir, extra_preargs=['-O3','-fPIC','-mavx'])
	if sys.platform.startswith('linux'):
		c.set_libraries(['rt','gomp'])
else:
	objects = c.compile([sourcefile], output_dir=objdir)

				
# create libraries
libdir = os.path.join(os.getcwd(),"jackal_n3_01_H10_noSlack","lib")
exportsymbols = ["%s_solve" % "jackal_n3_01_H10_noSlack", "%s_internal_mem" % "jackal_n3_01_H10_noSlack"]
c.create_static_lib(objects, "jackal_n3_01_H10_noSlack", output_dir=libdir)
c.link_shared_lib(objects, "jackal_n3_01_H10_noSlack", output_dir=libdir, export_symbols=exportsymbols)
