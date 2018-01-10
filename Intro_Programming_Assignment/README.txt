The time used by matmult.c, matmult_pure.py and matmult_npy.py in different sizes of input matrix is as follow:

									matmult.c	matmult_pure.py		matmult_npy.py
A and B are 10x10 matrices			17 us  		122 us 				104 us
A and B are 100x100 matrices		3624 us  	79355 us 			2195 us
A and B are 1000x1000 matrices		error		error				2314 us