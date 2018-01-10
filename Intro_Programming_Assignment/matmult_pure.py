import sys
import time
import csv

# function to check if an input string is integer.
def is_int(input_string):
    try: int(input_string)
    except ValueError: return 0
    else: return 1

def utime_now():
	return int(time.time()*1E6)

# read matrix from csv file name
def readMatrixFromFile(Filename):
	with open(Filename) as csvfile:
	    readCSV = csv.reader(csvfile)
	    mat = list(readCSV)
	    for i in range(len(mat)):
	    	for j in range(len(mat[0])):
	    		mat[i][j] = float(mat[i][j])
	    return mat

# write matrix to csv file name
def writeMatrixToFile(Filename, mat):
	with open(Filename, "w") as csvfile:
	    writer = csv.writer(csvfile, lineterminator='\n')
	    writer.writerows(mat)

# matrix multiplication
def matrix_multiply(mat_A, mat_B, row_C, col_C, num_mult):
	mat_C = []
	for i in range(row_C):
		row_mem = []
		for j in range(col_C):
			cal_val = 0
			for k in range(num_mult):
				cal_val += (mat_A[i][k])*(mat_B[k][j])
			row_mem.append(cal_val)
		mat_C.append(row_mem)
	return mat_C

if len(sys.argv) != 5:
	print "ERROR: expected four input arguments."
	sys.exit()

# read terminal input
row_A = sys.argv[1]
col_A = sys.argv[2]
row_B = sys.argv[3]
col_B = sys.argv[4]

# check valid input
if is_int(row_A) and is_int(col_A) and is_int(row_B) and is_int(col_B):
	row_A = int(row_A)
	col_A = int(col_A)
	row_B = int(row_B)
	col_B = int(col_B)
	if row_A <= 0 or col_A <= 0 or row_B <= 0 or col_B <= 0:
		print "ERROR: expected input arguments are positive more than and equal 1."
		sys.exit()
	if col_A != row_B:
		print "ERROR: expected matix dimensions are matched to multiply."
		sys.exit()
else:
	print "ERROR: expected input arguments are integers."
	sys.exit()

# read matrix from csv files
mat_A = readMatrixFromFile("A.csv")
mat_B = readMatrixFromFile("B.csv")

time_before = utime_now()

# C = A*B
mat_C = matrix_multiply(mat_A, mat_B, row_A, col_B, col_A)

time_after = utime_now()

# write calculated matrix to csv file
writeMatrixToFile("C.csv", mat_C)

print "operation time: %d us" %(time_after - time_before) 

