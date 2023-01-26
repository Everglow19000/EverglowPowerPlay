package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Sequence {
	public static class SequenceItem{
		public final HashSet<State.Message> messagesToWait;
		public final Runnable runAction;

		public SequenceItem(HashSet<State.Message> messagesToWait, Runnable runAction) {
			this.messagesToWait = messagesToWait;
			this.runAction = runAction;
		}

		public SequenceItem(State.Message message, Runnable runAction){
			this(new HashSet<>(Collections.singletonList(message)), runAction);
		}
	}

	private final Deque<SequenceItem> items;


	public Sequence(SequenceItem ...items){
		this.items = new ArrayDeque<>(Arrays.asList(items));
	}

	public void start(){
		items.getFirst().runAction.run();
		handleContinue();
	}

	public void interrupt(){
		items.clear();
	}

	public boolean isSequenceDone(){
		return items.isEmpty();
	}

	public void handleMessage(State.Message message){
		if (isSequenceDone()){
			return;
		}
		SequenceItem firstItem = items.getFirst();
		firstItem.messagesToWait.remove(message);
		handleContinue();
	}

	private void handleContinue(){
		while (true){
			if (isSequenceDone()) return;
			SequenceItem currentlyWaitingFor = items.getFirst();
			if(!currentlyWaitingFor.messagesToWait.isEmpty()){
				return;
			}
			items.removeFirst();
			if (isSequenceDone()) return;
			items.getFirst().runAction.run();
		}
	}
}
