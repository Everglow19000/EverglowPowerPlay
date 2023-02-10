package org.firstinspires.ftc.teamcode.utils.MLP;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

public class MLP {
	ArrayList<double[][]> weights;
	Activation activation;
	private final String WEIGHTS_FOLDER_NAME = "MLP";
	private final File WEIGHTS_FOLDER;

	public static final int IMG_SIZE = 28;

	public enum Activation {
		RELU, LOGISTIC
	}

	public MLP(Activation activation) throws IOException {
		this(new ArrayList<>(), activation);
	}

	public MLP(ArrayList<double[][]> weights, Activation activation) throws IOException {
		this.weights = weights;
		this.activation = activation;

		WEIGHTS_FOLDER = new File(AppUtil.FIRST_FOLDER, WEIGHTS_FOLDER_NAME);
		if (!WEIGHTS_FOLDER.exists() && !WEIGHTS_FOLDER.isDirectory()) {
			throw new IOException("Weights folder does not exist");
		}
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

	public double[] resizeAndFlatten(Mat mat) {
		Size newSize = new Size(IMG_SIZE,IMG_SIZE);
		Mat result = new Mat();
		Imgproc.resize(mat, result, newSize, 0, 0, Imgproc.INTER_AREA);

		Imgproc.cvtColor(result, result, Imgproc.COLOR_RGB2GRAY);
		result.inv();

		int total = 28 * 28; // total number of pixels
		Mat resMat = new Mat();
		result.reshape(1,total).convertTo(resMat, CvType.CV_32FC1);

		return resMat;
	}

	/**
	 * Load the MLP with weights from am already trained MLP.
	 * Weights are found in the weights folder in the form
	 * of csv files representing matrices.
	 * @throws IOException weights weren't found
	 */
	@RequiresApi(api = Build.VERSION_CODES.O)
	public void defaultLoad() throws IOException {
		int[][] sizes = new int[][]{{784, 784}, {784, 16}, {16, 16}, {16, 3}, {3, 3}};
		for (int i = 0; i < 5; i++) {
			// replace with path to weight csv-s within the Control Hub
			File[] files = WEIGHTS_FOLDER.listFiles();
			if (files == null || files.length < 1) throw new IOException("No weights found");

			for (File file:files) {
				if (!file.exists() || !file.isFile()) continue;
				weights.add(LinAlgUtils.readMatrixFromFile(file, sizes[i]));
			}
		}
	}
}