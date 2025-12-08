package com.chaigptrobotics.shenanigans;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Inherited
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Aura {

    int level() default 0;

    String[] auraLevels() default {"zeta", "gamma", "delta", "theta", "beta", "alpha", "omega", "giga", "tera", "sigma", "mew"};
    String[] auraStats() default {"GOOD", "LOCKED IN", "COOKING", "CRAZY", "+AURA", "W AURA", "INSANITY", "AURA FARMING", "AURA SWEATING", "AURA MAXING", "PEAK"};

}
