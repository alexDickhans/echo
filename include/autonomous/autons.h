#pragma once

/**
 * Elim to define different auton routines
 */
enum Auton_ {
  N_1_6,
  N_6,
  P_4,
  P_1_3GOAL,
  P_1_3ALLIANCE,
  P_1_1_2,
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

#define OPPONENTS (ALLIANCE == RED ? BLUE : RED)
