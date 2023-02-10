package org.firstinspires.ftc.teamcode.utils.MLP;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

public class LinAlgUtils {
	/**
	 * @param A a matrix
	 * @param x a vector
	 * @return Matrix-vector multiplication
	 */
	public static double[] dotAx(double[][] A, double[] x) {
		int n = x.length;
		int m = A[0].length;
		if (A.length != n) {
			throw new RuntimeException("Shape of A must match size of x");
		}
		double[] dot = new double[m];
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				dot[i] += A[j][i] * x[j];
			}
		}
		return dot;
	}

	public static double logistic(double z) {
		return 1 / (1 + Math.exp(-z));
	}

	public static double[] logistic(double[] z) {
		double[] res = new double[z.length];
		for (int i = 0; i < z.length; i++) {
			res[i] = logistic(z[i]);
		}
		return res;
	}

	public static double relu(double z) {
		return Math.max(0, z);
	}

	public static double[] relu(double[] z) {
		double[] res = new double[z.length];
		for (int i = 0; i < z.length; i++) {
			res[i] = relu(z[i]);
		}
		return res;
	}

	/**
	 * @param arr an array
	 * @return Index of the largest array element
	 */
	public static int argmax(double[] arr) {
		int index = 0;
		double value = arr[0];
		for (int i = 0; i < arr.length; i++) {
			if (arr[i] > value) {
				index = i;
				value = arr[i];
			}
		}
		return index;
	}

	/**
	 * Used to read the csv-s into arrays
	 * @param path Path of the csv file
	 * @param shape Supposed shape of the matrix
	 * @return 2D array (matrix) representation of the contents of the csv file
	 * @throws IOException
	 */
	@RequiresApi(api = Build.VERSION_CODES.O)
	public static double[][] readMatrixFromFile(Path path, int[] shape) throws IOException {
		List<String> arr = Files.readAllLines(path);
		String[] arr2;
		double[][] arr3 = new double[shape[0]][shape[1]];
		for (int i = 0; i < shape[0]; i++) {
			arr2 = arr.get(i).split(",");
			for (int j = 0; j < shape[1]; j++)
				arr3[i][j] = Double.parseDouble(arr2[j]);
		}
		return arr3;
	}

	/**
	 * Used to read a vector from a txt file
	 * @param path Path of the txt file
	 * @param size Size of the vector
	 * @return 1D array (vector) representation of the contents of the txt file
	 * @throws IOException
	 */
	@RequiresApi(api = Build.VERSION_CODES.O)
	public static double[] readVectorFromFile(Path path, int size) throws IOException {
//		String[] arr = Files.readAllLines(path).toString().split(",");
//		double[] arr2 = new double[size];
//		for (int i = 0; i < size; i++) {
//			arr2[i] = Double.parseDouble(arr[i]);
//		}
//		return arr2;
		return null;
	}
}