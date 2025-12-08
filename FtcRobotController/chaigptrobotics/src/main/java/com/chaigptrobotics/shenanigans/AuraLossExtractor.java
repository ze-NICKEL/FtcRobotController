package com.chaigptrobotics.shenanigans;

public class AuraLossExtractor {

    public String extract(AuraLoss auraLossAnnotation) {

        int level = auraLossAnnotation.level();

        StringBuilder auraLoss = new StringBuilder();

        for (int i = 0; i < level; i++) {
            auraLoss.append(EmojiArray.getEmoji(5)); // index 5 is the wilting flower symbol (🥀)
        }

        return auraLoss.toString();
    }
}
