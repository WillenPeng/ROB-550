import sys
import numpy


# function to check if an input string is integer.
def is_int(input_string):
    try: int(input_string)
    except ValueError: return 0
    else: return 1

if len(sys.argv) != 3:
	print "ERROR: expected two integer arguments"
	sys.exit()
# read terminal input
row_dim = sys.argv[1]
col_dim = sys.argv[2]

# check valid input
if is_int(row_dim) and is_int(col_dim):
	row_dim = int(row_dim)
	col_dim = int(col_dim)
	if row_dim <= 0 or col_dim <= 0:
		print "ERROR: expected two integer arguments"
		sys.exit()
else:
	print "ERROR: expected two integer arguments"
	sys.exit()

# generate normally distributed random matrix 
mat_output = numpy.random.randn(row_dim,col_dim)

for row in range(0,row_dim):
	row_str = [str(i) for i in mat_output[row]]
	print ', '.join(row_str)



