package com.chaigptrobotics.shenanigans;

public class AuraExtractor {

    public String[] extract(Aura auraAnnotation) {

        int level = auraAnnotation.level();

        String auraHeader = auraAnnotation.auraLevels()[level];
        String auraStat = auraAnnotation.auraStats()[level];

        return new String[] {auraHeader, auraStat};
    }
}