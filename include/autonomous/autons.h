#pragma once

/**
 * Elim to define different auton routines
 */
enum Auton_ {
  AWP_PUSH,
  SAWP,
  POS_ELIM,
  NEG_ELIM,
  NEG_ELIM_NO_ALLIANCE,
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
