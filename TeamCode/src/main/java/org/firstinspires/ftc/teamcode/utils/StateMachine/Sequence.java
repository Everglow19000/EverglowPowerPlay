package org.firstinspires.ftc.teamcode.utils.StateMachine;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.HashSet;

/**
 * A class for creating a sequence of actions.
 * Also contains the SequenceItem class, which is used to create an action in a sequence.
 */
public class Sequence {
	/**
	 * A class for storing an action in a sequence.
	 * Takes an action in the form of a Runnable.
	 */
	public static class SequenceItem {
		public final HashSet<StateMessages> messagesToWait;
		public final Runnable runAction;

		public SequenceItem(HashSet<StateMessages> messagesToWait, Runnable runAction) {
			this.messagesToWait = messagesToWait;
			this.runAction = runAction;
		}

		public SequenceItem(StateMessages message, Runnable runAction) {
			this(new HashSet<>(Collections.singletonList(message)), runAction);
		}
	}

	/**
	 * The items in the sequence.
	 */
	private final Deque<SequenceItem> items;

	/**
	 * Assigns an array of sequence items created previously to the sequence.
	 *
	 * @param items The items in the sequence.
	 */
	public Sequence(SequenceItem... items) {
		this.items = new ArrayDeque<>(Arrays.asList(items));
	}

	/**
	 * Starts the sequence by running the first action and then handling the rest of the sequence.
	 */
	public void start() {
		items.getFirst().runAction.run();
		handleContinue();
	}

	/**
	 * Interrupts the sequence by clearing the items in the sequence, making it stop executing.
	 */
	public void interrupt() {
		items.clear();
	}

	public boolean isSequenceDone() {
		return items.isEmpty();
	}

	/**
	 * Handles receiving messages from the System Coordinator.
	 * If the message is in the first item in the sequence, it is removed from the list of messages to wait for.
	 *
	 * @param message The message to be handled.
	 */
	public void handleMessage(StateMessages message) {
		if (isSequenceDone()) {
			return;
		}
		SequenceItem firstItem = items.getFirst();
		firstItem.messagesToWait.remove(message);
		handleContinue();
	}

	/**
	 * Handles executing the rest of the sequence, including waiting for messages.
	 */
	private void handleContinue() {
		while (true) {
			if (isSequenceDone()) return;
			SequenceItem currentlyWaitingFor = items.getFirst();
			if (!currentlyWaitingFor.messagesToWait.isEmpty()) {
				return;
			}
			items.removeFirst();
			if (isSequenceDone()) return;
			items.getFirst().runAction.run();
		}
	}
}
