#pragma once

/**
 * Elim to define different auton routines
 */
enum Auton_ {
  AWP,
  AWP_PUSH,
  POS_ELIM,
  NEG_ELIM,
  SKILLS,
  NONE
} typedef Auton;

/**
 * Defines which alliance we are on
 */
enum Alliance_ {
  RED = 1,
  BLUE = 2
} typedef Alliance;
