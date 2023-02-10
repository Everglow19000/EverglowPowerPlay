package org.firstinspires.ftc.teamcode.utils.MLP;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

public class MLP {
	ArrayList<double[][]> weights;
	Activation activation;

	public enum Activation {
		RELU, LOGISTIC
	}

	public MLP(Activation activation) {
		this.weights = new ArrayList<>();
		this.activation = activation;
	}

	public MLP(ArrayList<double[][]> weights, Activation activation) {
		this.weights = weights;
		this.activation = activation;
	}

	/**
	 * Predict the label of an img
	 * @param img 784-long vector representing a flattened 28*28 img
	 * @return Array of probabilities for each label
	 */
	public double[] forward(double[] img) {
		for (double[][] weight : weights) {
			if (activation == Activation.RELU) {
				img = LinAlgUtils.relu(LinAlgUtils.dotAx(weight, img));
			} else {
				img = LinAlgUtils.logistic(LinAlgUtils.dotAx(weight, img));
			}
		}
		return img;
	}

	/**
	 * Load the MLP with weights from am already trained MLP.
	 * Weights are found in the weights folder in the form
	 * of csv files representing matrices.
	 * @throws IOException
	 */
	@RequiresApi(api = Build.VERSION_CODES.O)
	public void defaultLoad() throws IOException {
		int[][] sizes = new int[][]{{784, 784}, {784, 16}, {16, 16}, {16, 3}, {3, 3}};
		for (int i = 0; i < 5; i++) {
			// replace with path to weight csv-s within the Control Hub
			Path path = Paths.get("weights/weights0.csv");
			weights.add(LinAlgUtils.readMatrixFromFile(path, sizes[i]));
		}
	}
}