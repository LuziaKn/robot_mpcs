import numpy
import ctypes

name = "jackal_n3_01_H10_noSlack"
requires_callback = True
lib = "lib/libjackal_n3_01_H10_noSlack.so"
lib_static = "lib/libjackal_n3_01_H10_noSlack.a"
c_header = "include/jackal_n3_01_H10_noSlack.h"
nstages = 10

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (100,   1),  100),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  8,   1),    8),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (270,   1),  270)]

# Output                | Type    | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x02"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x03"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x04"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x05"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x06"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x07"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x08"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x09"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x10"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
 ('it2opt', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('res_ineq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('rcompnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('dobj', ctypes.c_double),
 ('dgap', ctypes.c_double),
 ('rdgap', ctypes.c_double),
 ('mu', ctypes.c_double),
 ('mu_aff', ctypes.c_double),
 ('sigma', ctypes.c_double),
 ('lsit_aff', ctypes.c_int),
 ('lsit_cc', ctypes.c_int),
 ('step_aff', ctypes.c_double),
 ('step_cc', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('solver_id', ctypes.c_int * 8)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(10, 8, 11, 27, 2, 2, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 8, 11, 27, 10, 10, 11, 0), 
	(10, 0, 11, 27, 10, 10, 11, 0)
]