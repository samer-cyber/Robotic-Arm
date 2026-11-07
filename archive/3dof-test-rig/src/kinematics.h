#pragma once
#include "config.h"
#include <math.h>

// ===========================================================================
//  3R articulated arm:  J1 = base yaw, J2 = shoulder pitch, J3 = elbow pitch.
//
//  Angle convention (change here if your mechanics disagree):
//    q1  0 deg = +X axis, CCW positive looking down at the base
//    q2  0 deg = upper arm horizontal, positive = lifting up
//    q3  0 deg = forearm in line with the upper arm, RELATIVE to link 1,
//                negative = folding the elbow upward
//
//  When you add a spherical wrist (J4/J5/J6) this whole block gets replaced
//  by wrist-centre decoupling: subtract the tool offset along the approach
//  vector to get the wrist centre, solve the code below for that point, then
//  solve the wrist from the residual rotation. The rest of the firmware
//  talks to kinematics only through fkPose()/ikSolve(), so nothing else moves.
// ===========================================================================

struct Pose   { float x, y, z; };
struct Joints { float q[MAX_AXES]; };

enum IkStatus { IK_OK = 0, IK_UNREACHABLE, IK_OUT_OF_LIMITS };

inline const char* ikStatusStr(IkStatus s) {
  switch (s) {
    case IK_OK:              return "ok";
    case IK_UNREACHABLE:     return "unreachable";
    case IK_OUT_OF_LIMITS:   return "violates joint limits";
  }
  return "?";
}

// NOTE since the wrist exists: GEO.l2 is now J3 -> J4, so fkPose()/ikSolve()
// resolve the J4 AXIS - the wrist root - not the tool tip. That is the correct
// decoupling for a 6-DOF arm: place the wrist centre with J1..J3, then set
// orientation with J4..J6. For the tool tip (and anything to do with hitting
// the desk) use fkChain() at the bottom of this file, which walks the servos too.

// ---- forward kinematics: joint angles (deg) -> WRIST ROOT (J4 axis), mm ---
inline void fkPose(const float q[], Pose &p) {
  const float t1 = radians(q[0]);
  const float t2 = radians(q[1]);
  const float t3 = radians(q[2]);
  const float r  = GEO.baseOffset + GEO.l1 * cosf(t2) + GEO.l2 * cosf(t2 + t3);
  p.z = GEO.baseHeight + GEO.l1 * sinf(t2) + GEO.l2 * sinf(t2 + t3);
  p.x = r * cosf(t1);
  p.y = r * sinf(t1);
}

// ---- inverse kinematics: tool point (mm) -> joint angles (deg) ------------
// Closed form, two branches. `checkLimits` off lets you inspect a solution
// that the soft limits would otherwise reject.
inline IkStatus ikSolve(const Pose &p, bool elbowUp, float q[], bool checkLimits = true) {
  const float t1 = atan2f(p.y, p.x);
  const float r  = sqrtf(p.x * p.x + p.y * p.y) - GEO.baseOffset;
  const float z  = p.z - GEO.baseHeight;

  const float L1 = GEO.l1, L2 = GEO.l2;
  float c3 = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  if (c3 >  1.0f) { if (c3 >  1.0005f) return IK_UNREACHABLE; c3 =  1.0f; }
  if (c3 < -1.0f) { if (c3 < -1.0005f) return IK_UNREACHABLE; c3 = -1.0f; }

  float t3 = acosf(c3);
  if (elbowUp) t3 = -t3;
  const float t2 = atan2f(z, r) - atan2f(L2 * sinf(t3), L1 + L2 * cosf(t3));

  q[0] = degrees(t1);
  q[1] = degrees(t2);
  q[2] = degrees(t3);
  for (uint8_t i = 3; i < MAX_AXES; i++) q[i] = 0.0f;

  if (checkLimits) {
    for (uint8_t i = 0; i < 3 && i < N_AXES; i++)
      if (q[i] < AX[i].minDeg || q[i] > AX[i].maxDeg) return IK_OUT_OF_LIMITS;
  }
  return IK_OK;
}

// ---- reach envelope, handy for generating valid random test points --------
inline float reachMin() { return fabsf(GEO.l1 - GEO.l2); }
inline float reachMax() { return GEO.l1 + GEO.l2; }


// ===========================================================================
//  FULL CHAIN, including the three wrist servos.
//
//  Sign convention, all pitches: POSITIVE LIFTS. q2, q3, q4 and q6 all follow
//  it. q5 is the roll; its axis lies along the link, so rolling moves nothing
//  by itself - it decides which way J6's pitch plane is tilted. If a servo
//  turns the wrong way, set `invert` on it rather than editing this.
//
//  J4 -> J5 (l3) and J5 -> J6 (l4) are taken as collinear with the forearm
//  direction. That holds for the current wrist; if you add a dogleg, this is
//  the function that needs the extra offset.
// ===========================================================================
struct ArmPoints { Pose j2, j3, j4, j5, j6, tip; };

// (radial, out-of-plane, z) + base yaw -> world xyz
inline Pose _yaw(float r, float sOut, float z, float c1, float s1) {
  Pose p;
  p.x = r * c1 - sOut * s1;
  p.y = r * s1 + sOut * c1;
  p.z = z;
  return p;
}

// q[0..2] = J1,J2,J3 in deg.  sv[0..2] = J4,J5,J6 servo angles in deg.
inline void fkChain(const float q[], const float sv[], ArmPoints &P) {
  const float c1 = cosf(radians(q[0])), s1 = sinf(radians(q[0]));
  const float a2 = radians(q[1]);
  const float a3 = a2 + radians(q[2]);
  const float a4 = a3 + radians(sv[0]);
  const float q5 = radians(sv[1]);
  const float q6 = radians(sv[2]);

  float r = GEO.baseOffset,  z = GEO.baseHeight;
  P.j2 = _yaw(r, 0.0f, z, c1, s1);
  r += GEO.l1 * cosf(a2);  z += GEO.l1 * sinf(a2);   P.j3 = _yaw(r, 0.0f, z, c1, s1);
  r += GEO.l2 * cosf(a3);  z += GEO.l2 * sinf(a3);   P.j4 = _yaw(r, 0.0f, z, c1, s1);
  r += GEO.l3 * cosf(a4);  z += GEO.l3 * sinf(a4);   P.j5 = _yaw(r, 0.0f, z, c1, s1);
  r += GEO.l4 * cosf(a4);  z += GEO.l4 * sinf(a4);   P.j6 = _yaw(r, 0.0f, z, c1, s1);

  // Tip leaves the vertical plane as soon as BOTH the roll and J6 are off zero.
  // u = link direction, n = J6's pitch axis after the roll, tip = u pitched by q6.
  const float ca4 = cosf(a4), sa4 = sinf(a4);
  const float c5 = cosf(q5),  s5 = sinf(q5);
  const float c6 = cosf(q6),  s6 = sinf(q6);
  const float nxu_r = sa4 * c5, nxu_s = s5, nxu_z = -ca4 * c5;   // n x u
  const float dr = ca4 * c6 - nxu_r * s6;
  const float ds =       0.0f - nxu_s * s6;
  const float dz = sa4 * c6 - nxu_z * s6;
  P.tip = _yaw(r + GEO.l5 * dr, GEO.l5 * ds, z + GEO.l5 * dz, c1, s1);
}

// Lowest point of anything the firmware knows the position of. The tip is
// usually it, but a folded elbow can dip below the wrist, so check them all.
inline float lowestZ(const ArmPoints &P) {
  float m = P.j2.z;
  if (P.j3.z  < m) m = P.j3.z;
  if (P.j4.z  < m) m = P.j4.z;
  if (P.j5.z  < m) m = P.j5.z;
  if (P.j6.z  < m) m = P.j6.z;
  if (P.tip.z < m) m = P.tip.z;
  return m;
}

// How much room is left under the arm. Negative = already through the desk.
inline float deskClearance(const ArmPoints &P) {
  return lowestZ(P) - (GEO.deskZ + GEO.deskMargin);
}
inline bool deskOK(const ArmPoints &P) { return deskClearance(P) >= 0.0f; }
