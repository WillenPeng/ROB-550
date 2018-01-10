#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

double* matrix_multiply(double *, double *, int, int, int);
double* readMatrixFromFile(char *, int, int);
int writeMatrixToFile(char *, double *, int, int);
int64_t utime_now (void);


int main(int argc, char *argv[]){
    if (argc != 5){
    	fprintf(stderr, "ERROR: expected four input arguments.\n");
        exit(1);
    }
    // read input
    double row_A = atof(argv[1]);
    double col_A = atof(argv[2]);
    double row_B = atof(argv[3]);
    double col_B = atof(argv[4]);
    // check if inputs are integer
    if (row_A != floor(row_A) || col_A != floor(col_A) || row_B != floor(row_B) || col_B != floor(col_B)){
    	fprintf(stderr, "ERROR: expected input arguments are integers.\n");
        exit(1);
    }
    // check positive
    if (row_A <= 0 || col_A <= 0 || row_B <= 0 || col_B <= 0){
    	fprintf(stderr, "ERROR: expected input arguments are positive more than and equal 1.\n");
        exit(1);
    }
    // check matched matrix dimensions
    if (col_A != row_B){
    	fprintf(stderr, "ERROR: expected matix dimensions are matched to multiply.\n");
        exit(1);
    }
    int row_dim_A = row_A;
    int col_dim_A = col_A;
    int row_dim_B = row_B;
    int col_dim_B = col_B;

    int row_dim_C = row_dim_A;
    int col_dim_C = col_dim_B;

    // read matrix from csv files
    char *inFileString_A, *inFileString_B, *outFileString;

    inFileString_A = "A.csv";
    inFileString_B = "B.csv";
    outFileString = "C.csv";

    double* mat_A = readMatrixFromFile(inFileString_A, row_dim_A, col_dim_A);
    double* mat_B = readMatrixFromFile(inFileString_B, row_dim_B, col_dim_B);
	// C = A*B
	int64_t time_before = utime_now();
    double* mat_C = matrix_multiply(mat_A, mat_B, row_dim_C, col_dim_C, col_dim_A);
    int64_t time_after = utime_now();
    printf("operation time: %ld us\n", time_after - time_before);
    // write final matrix to csv file
    writeMatrixToFile(outFileString, mat_C, row_dim_C, col_dim_C);

    return(0);
    
}

// matrix multiplication
double* matrix_multiply(double* mat_A, double* mat_B, int row_final, int col_final, int num_mult) {
	double* mat_C = (double*) malloc(row_final * col_final * sizeof(double));
	int i,j,k;
    for (i = 0; i < row_final; i++){
        for (j = 0; j < col_final; j++){
            mat_C[i*col_final + j] = 0;
            for (k = 0; k < num_mult; k++){
               mat_C[i*col_final + j] += mat_A[i*num_mult + k]*mat_B[k*col_final + j];
            }
        }
    }

    return mat_C;
}

int64_t utime_now (void){
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

/**
 * Allocates a matrix which must be free'd by the calling code.
 * i.e. free()
 */
double* readMatrixFromFile(char* fileName, int height, int width) {
  FILE* fp = fopen(fileName, "r");
  if (fp == NULL) {
	fprintf(stderr, "Can't open %s.\n", fileName);
	return NULL;
  }
  double val;
  double* M = (double*) malloc(height * width * sizeof(double));
  int i,j;
  for(i = 0; i < height; i++) {
	for(j = 0; j < width; j++) {
  	if (fscanf(fp, " %lf", &val) != 1) {
    	fprintf(stderr, "Couldn't read value.\n");
    	return NULL;
  	}
  	// Discard the comma without checking.
  	fgetc(fp);
  	M[i * width + j] = val;
	}
  }
  fclose(fp);
  return M;
}

int writeMatrixToFile(char* fileName, double* matrix, int height, int width) {
  FILE* fp = fopen(fileName, "w");
  if (fp == NULL) {
	return 1;
  }
  int i,j;
  for (i = 0; i < height; i++) {
	for (j = 0; j < width; j++) {
  	if (j > 0) {
    	fputc(',', fp);
  	}
  	fprintf(fp, "%lf", matrix[i*width +j]);
	}
	fputs("\r\n", fp);
  }
  fclose(fp);
  return 0;
}
