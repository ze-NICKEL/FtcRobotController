package com.chaigptrobotics.shenanigans;

public class EmojiArray {

    /// An array of chosen emojis as strings - somehow they count as more than one character
    public static String[] emojis = {
            "\uD83E\uDD26\u200D♂\uFE0F", //🤦‍♂️
            "\uD83E\uDD23", //🤣
            "\uD83D\uDC4C", //👌
            "\uD83E\uDD23", //🤣
            "\uD83D\uDE3C", //😼
            "\uD83E\uDD40", //🥀
            "\uD83D\uDC4D", //👍
            "\uD83D\uDC4E", //👎
            "⛈\uFE0F", //⛈️
            "\uD83E\uDE74", //🩴
            "\uD83E\uDEE1", //🫡
            "\uD83E\uDD49", //🥉
            "\uD83E\uDD48", //🥈
            "\uD83E\uDD47", //🥇
            "\uD83D\uDDFF", //🗿
            "\uD83D\uDC10", //🐐
            "\uD83D\uDD25", //🔥
            "\uD83E\uDD0C", //🤌
            "\uD83E\uDD28", //🤨
            "\uD83D\uDC80", //💀
            "\uD83E\uDD11", //🤑
            "\uD83D\uDCB7", //💷
            "\uD83D\uDC8E", //💎
            "\uD83D\uDD75\uFE0F\u200D♂\uFE0F", //🕵️‍♂️
            "\uD83E\uDDA8", //🦨
            "‼\uFE0F", //‼️
            "✅" //✅
    };

    public static String getEmoji(int index) {
        return emojis[index];
    }
}
