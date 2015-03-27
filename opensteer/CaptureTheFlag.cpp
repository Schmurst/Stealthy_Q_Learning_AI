// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// Capture the Flag   (a portion of the traditional game)
//
// The "Capture the Flag" sample steering problem, proposed by Marcin
// Chady of the Working Group on Steering of the IGDA's AI Interface
// Standards Committee (http://www.igda.org/Committees/ai.htm) in this
// message (http://sourceforge.net/forum/message.php?msg_id=1642243):
//
//     "An agent is trying to reach a physical location while trying
//     to stay clear of a group of enemies who are actively seeking
//     him. The environment is littered with obstacles, so collision
//     avoidance is also necessary."
//
// Note that the enemies do not make use of their knowledge of the 
// seeker's goal by "guarding" it.  
//
// XXX hmm, rename them "attacker" and "defender"?
//
// 08-12-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "Q_Learner.h"

#define M_PI 3.14159265359f

using namespace OpenSteer;

// ----------------------------------------------------------------------------
// short names for STL vectors (iterators) of SphericalObstacle pointers


typedef std::vector<SphericalObstacle*> SOG;  // spherical obstacle group
typedef SOG::const_iterator SOI;              // spherical obstacle iterator


// ----------------------------------------------------------------------------
// This PlugIn uses two vehicle types: CtfSeeker and CtfEnemy.  They have a
// common base class: CtfBase which is a specialization of SimpleVehicle.


class CtfBase : public SimpleVehicle
{
public:
  // constructor
  CtfBase() { reset(); }

  // reset state
  void reset(void);

  // draw this character/vehicle into the scene
  void draw(void);

  // annotate when actively avoiding obstacles
  void annotateAvoidObstacle(const float minDistanceToCollision);

  void drawHomeBase(void);

  void randomizeStartingPositionAndHeading(void);
  // Add hiding to the state enum.
  enum seekerState { running, hiding, fleeing, tagged, atGoal };

  // for draw method
  Vec3 bodyColor;

  // xxx store steer sub-state for anotation
  bool avoiding;

  // dynamic obstacle registry
  static void initializeObstacles(void);
  static void addOneObstacle(void);
  static void removeOneObstacle(void);
  float minDistanceToObstacle(const Vec3 point);
  static int obstacleCount;
  static const int maxObstacleCount;
  static SOG allObstacles;
};


class CtfSeeker : public CtfBase
{
public:

  // Q Learner
  Q_Learner Q_agent;
  Q_Learner::worldState* get_world_state();
  Q_Learner::worldState* prev_world_state;
  Q_Learner::worldState* curr_world_state;
  float QL_timer = 0.0f;
  float QL_episode_length = 1.0f; // seconds

  // constructor
  CtfSeeker() {
    reset();
    Q_agent.init();
  }

  // reset state
  void reset(void);

  // per frame simulation update
  void update(const float currentTime, const float elapsedTime);

  // is there a clear path to the goal?
  bool clearPathToGoal(const Vec3 target);

  // hide behaviour
  Vec3 hide();
  Vec3 Q_Learning_Steering();

  Vec3 steeringForSeeker(void);
  void updateState(const float currentTime);
  void draw(void);
  Vec3 steerToEvadeAllDefenders(void);
  Vec3 XXXsteerToEvadeAllDefenders(void);
  void adjustObstacleAvoidanceLookAhead(const bool clearPath);
  void clearPathAnnotation(const float threshold,
    const float behindcThreshold,
    const Vec3& goalDirection);

  // hiding behaviour (Sam Hayhurst)
  Vec3 GetHidingPosition(const Vec3 posOb, const float radiusOb, const Vec3 posTarget);

  seekerState state;
  bool evading; // xxx store steer sub-state for anotation
  float lastRunningTime; // for auto-reset
};


class CtfEnemy : public CtfBase
{
public:

  // AI seeking variables
  float viewingAngle = 0.52f; // 30 degrees: 60 degree arc
  float viewingRadii = 15.0f;
  int wanderCounter = 0;
  Vec3 wanderSteer;
  int num_rays = 7; // should be odd inoder to have central ray
  // constructor
  CtfEnemy() { reset(); }

  // reset state
  void reset(void);

  //checking fucntion to return whether the seeker has been spotted
  bool is_seeker_spotted();

  // per frame simulation update
  void update(const float currentTime, const float elapsedTime);
};

#pragma region GLOBALS
// ----------------------------------------------------------------------------
// globals
// (perhaps these should be member variables of a Vehicle or PlugIn class)

bool IS_DEBUG = false;
const int CtfBase::maxObstacleCount = 100;

const Vec3 gHomeBaseCenter(0, 0, 0);
const float gHomeBaseRadius = 1.5;

const float gMinStartRadius = 30;
const float gMaxStartRadius = 40;

const float gBrakingRate = 0.75;

const Vec3 evadeColor(0.6f, 0.6f, 0.3f); // annotation
const Vec3 seekColor(0.3f, 0.6f, 0.6f); // annotation
const Vec3 clearPathColor(0.3f, 0.6f, 0.3f); // annotation

const float gAvoidancePredictTimeMin = 0.9f;
const float gAvoidancePredictTimeMax = 2;
float gAvoidancePredictTime = gAvoidancePredictTimeMin;

bool enableAttackSeek = true; // for testing (perhaps retain for UI control?)
bool enableAttackEvade = true; // for testing (perhaps retain for UI control?)

CtfSeeker* gSeeker = NULL;


// count the number of times the simulation has reset (e.g. for overnight runs)
int resetCount = 0;

#pragma endregion

// ----------------------------------------------------------------------------
// state for OpenSteerDemo PlugIn
//
// XXX consider moving this inside CtfPlugIn
// XXX consider using STL (any advantage? consistency?)


CtfSeeker* ctfSeeker;
const int ctfEnemyCount = 1;
CtfEnemy* ctfEnemies[ctfEnemyCount];

#pragma region RESET_FUNCTIONS
// ----------------------------------------------------------------------------
// reset state


void CtfBase::reset(void)
{
  SimpleVehicle::reset();  // reset the vehicle 

  setSpeed(3);             // speed along Forward direction.
  setMaxForce(3.0);        // steering force is clipped to this magnitude
  setMaxSpeed(3.0);        // velocity is clipped to this magnitude

  avoiding = false;         // not actively avoiding

  randomizeStartingPositionAndHeading();  // new starting position

  clearTrailHistory();     // prevent long streaks due to teleportation
}


void CtfSeeker::reset(void)
{
  CtfBase::reset();
  bodyColor.set(0.4f, 0.4f, 0.6f); // blueish
  gSeeker = this;
  state = running;
  evading = false;
  prev_world_state = NULL;
  curr_world_state = NULL;
}


void CtfEnemy::reset(void)
{
  CtfBase::reset();
  bodyColor.set(0.6f, 0.4f, 0.4f); // redish

  // set speed to faster than that of the seeker set its starting pos to
  // that of the target disc


  setSpeed(5.0);
  setMaxForce(5.0);
  setMaxSpeed(5.0);

  // set starting pos to seeker's target
  setPosition(gHomeBaseCenter);
}

#pragma endregion

// ----------------------------------------------------------------------------
// draw this character/vehicle into the scene

void CtfBase::draw(void)
{
  drawBasic2dCircularVehicle(*this, bodyColor);
  drawTrail();
}

// ----------------------------------------------------------------------------

void CtfBase::randomizeStartingPositionAndHeading(void)
{
  // randomize position on a ring between inner and outer radii
  // centered around the home base
  const float rRadius = frandom2(gMinStartRadius, gMaxStartRadius);
  const Vec3 randomOnRing = RandomUnitVectorOnXZPlane() * rRadius;
  setPosition(gHomeBaseCenter + randomOnRing);

  // are we are too close to an obstacle?
  if (minDistanceToObstacle(position()) < radius() * 5)
  {
    // if so, retry the randomization (this recursive call may not return
    // if there is too little free space)
    randomizeStartingPositionAndHeading();
  }
  else
  {
    // otherwise, if the position is OK, randomize 2D heading
    randomizeHeadingOnXZPlane();
  }
}

// ----------------------------------------------------------------------------

void CtfEnemy::update(const float currentTime, const float elapsedTime)
{
  // determine upper bound for pursuit prediction time
  const float seekerToGoalDist = Vec3::distance(gHomeBaseCenter,
    gSeeker->position());
  const float adjustedDistance = seekerToGoalDist - radius() - gHomeBaseRadius;
  const float seekerToGoalTime = ((adjustedDistance < 0) ?
    0 :
    (adjustedDistance / gSeeker->speed()));
  const float maxPredictionTime = seekerToGoalTime * 0.9f;

  // determine steering (pursuit, obstacle avoidance, or braking)
  Vec3 steer(0, 0, 0);
  if (gSeeker->state != tagged || gSeeker->state != atGoal){
    const Vec3 avoidance =
      steerToAvoidObstacles(gAvoidancePredictTimeMin,
      (ObstacleGroup&)allObstacles);

    // saved for annotation
    avoiding = (avoidance == Vec3::zero);

    if (avoiding){
      float dist = (this->position() - gSeeker->position()).length();
      if (dist > 20.0f){
        steer = steerForSeek(gSeeker->position());
      }
      else if (wanderCounter == 0){
        steer = steerForWander(0.5f);
        steer.y = 0;
        wanderSteer = steer;
        wanderCounter += 30;
      }
      else{
        steer = wanderSteer;
        wanderCounter--;
      }
    }
    else
      steer = avoidance;
  }
  else
  {
    applyBrakingForce(gBrakingRate, elapsedTime);
  }
  applySteeringForce(steer, elapsedTime);

  // annotation
  if (!IS_DEBUG) annotationVelocityAcceleration();
  if (!IS_DEBUG) recordTrailVertex(currentTime, position());
  // viewing arc annotation


  for (unsigned int ray_index = 0; ray_index < num_rays; ++ray_index){
    // first iterate the angle of the ray being annotated
    float angle = viewingAngle - ray_index * viewingAngle / (num_rays / 2);
    Vec3 ray_end = position() + forward().rotateAboutGlobalY(angle) * viewingRadii;

    for (unsigned int index = 0; index < allObstacles.size(); ++index){
      // if obst center is within radius + viewing radius then do further checks
      Vec3 center = allObstacles[index]->center;
      float radii = allObstacles[index]->radius;
      Vec3 toObst = center - this->position();
      if (toObst.length() < viewingRadii + allObstacles[index]->radius){
        // check whether one of the rays connects with the obstacle
        SimpleVehicle::get_line_circle_intersection(ray_end, position(), ray_end, center, radii);
      }
    }
    if (!IS_DEBUG) annotationLine(position(), ray_end, Vec3(0, 1, 0));
  }


  if (is_seeker_spotted() && gSeeker->state != atGoal){
    gSeeker->state = tagged;
  }

  // annotation:
  if (gSeeker->state == tagged)
  {
    const Vec3 color(0.8f, 0.5f, 0.5f);
    if (!IS_DEBUG) annotationXZDisk(1.0f, gSeeker->position(), color, 20);
  }
}

bool CtfEnemy::is_seeker_spotted(){
  // detect and record interceptions ("tags") of seeker
  const float seekerToMeDist = (position() - gSeeker->position()).length();
  Vec3 toTarget = Vec3(gSeeker->position() - this->position()).normalize();
  bool seeker_spotted = false;
  Vec3 intersection_point;

  // is the seeker inside of me (wehay!) if so, set it to tagged state
  if (seekerToMeDist < this->radius()){
    seeker_spotted = true;
  }

  // is the seeker inside vision cone and radii?
  float angle_to_target = atan2f(toTarget.x, toTarget.z);
  float angle_to_heading = atan2f(getForward().x, getForward().z);
  if (angle_to_target < 0.0f) angle_to_target += 2 * M_PI;
  if (angle_to_heading < 0.0f) angle_to_heading += 2 * M_PI;
  float angle = angle_to_target - angle_to_heading;
  if (angle < 0.0f){
    angle = angle_to_heading - angle_to_target;
  }

  //printf("angle to target: %f\n", angle_to_target);
  //printf("angle to heading: %f\n", angle_to_heading);
  //printf("angle difference: %f\n\n", angle);

  Vec3 ray_end = position() + forward().rotateAboutGlobalY(angle) * 3.0f;
  if (!IS_DEBUG) annotationLine(position(), ray_end, Vec3(1, 0, 0));

  const bool inVisionCone = (angle < viewingAngle || angle > -viewingAngle + 2 * M_PI);
  if (seekerToMeDist < viewingRadii && inVisionCone)
  {
    if (gSeeker->state != tagged && gSeeker->state != atGoal){
      seeker_spotted = true;
      // now to test whether there is an obstacle in the way
      for (unsigned int index = 0; index < allObstacles.size(); ++index){
        // if obst center is within radius + viewing radius then do further checks
        Vec3 center = allObstacles[index]->center;
        float radii = allObstacles[index]->radius;
        Vec3 toObst = center - position();
        if ((toObst.length() + allObstacles[index]->radius) < viewingRadii){
          // check whether one of the rays connects with the player then check whether it first
          // connects with the sphere.
          for (unsigned int ray_index = 0; ray_index < num_rays; ++ray_index){
            float angle = viewingAngle - ray_index * viewingAngle / num_rays;
            Vec3 ray_end = position() + forward().rotateAboutGlobalY(angle) * viewingRadii;
            SimpleVehicle::get_line_circle_intersection(intersection_point, position(), ray_end, center, radii);
            if ((intersection_point - position()).lengthSquared() < (gSeeker->position() - position()).lengthSquared()){
              seeker_spotted = false;
            }
          }
        }
      }
    }
  }

  return seeker_spotted;
}


// ----------------------------------------------------------------------------
// are there any enemies along the corridor between us and the goal?


bool CtfSeeker::clearPathToGoal(const Vec3 target)
{
  const float sideThreshold = radius() * 8.0f;
  const float behindThreshold = radius() * 2.0f;

  const Vec3 goalOffset = target - position();
  const float goalDistance = goalOffset.length();
  const Vec3 goalDirection = goalOffset / goalDistance;

  const bool goalIsAside = isAside(gHomeBaseCenter, 0.5);

  // for annotation: loop over all and save result, instead of early return 
  bool xxxReturn = true;

  // loop over enemies
  for (int i = 0; i < ctfEnemyCount; i++)
  {
    // short name for this enemy
    const CtfEnemy& e = *ctfEnemies[i];
    const float eDistance = Vec3::distance(position(), e.position());
    const float timeEstimate = 0.3f * eDistance / e.speed(); //xxx
    const Vec3 eFuture = e.predictFuturePosition(timeEstimate);
    const Vec3 eOffset = eFuture - position();
    const float alongCorridor = goalDirection.dot(eOffset);
    const bool inCorridor = ((alongCorridor > -behindThreshold) &&
      (alongCorridor < goalDistance));
    const float eForwardDistance = forward().dot(eOffset);

    // xxx temp move this up before the conditionals
    if (!IS_DEBUG) annotationXZCircle(e.radius(), eFuture, clearPathColor, 20); //xxx

    // consider as potential blocker if within the corridor
    if (inCorridor)
    {
      const Vec3 perp = eOffset - (goalDirection * alongCorridor);
      const float acrossCorridor = perp.length();
      if (acrossCorridor < sideThreshold)
      {
        // not a blocker if behind us and we are perp to corridor
        const float eFront = eForwardDistance + e.radius();

        //annotationLine (position, forward*eFront, gGreen); // xxx
        //annotationLine (e.position, forward*eFront, gGreen); // xxx

        // xxx
        // std::ostringstream message;
        // message << "eFront = " << std::setprecision(2)
        //         << std::setiosflags(std::ios::fixed) << eFront << std::ends;
        // draw2dTextAt3dLocation (*message.str(), eFuture, gWhite);

        const bool eIsBehind = eFront < -behindThreshold;
        const bool eIsWayBehind = eFront < (-2 * behindThreshold);
        const bool safeToTurnTowardsGoal =
          ((eIsBehind && goalIsAside) || eIsWayBehind);

        if (!safeToTurnTowardsGoal)
        {
          // this enemy blocks the path to the goal, so return false
          if (!IS_DEBUG) annotationLine(position(), e.position(), clearPathColor);
          // return false;
          xxxReturn = false;
        }
      }
    }
  }

  // no enemies found along path, return true to indicate path is clear
  // clearPathAnnotation (sideThreshold, behindThreshold, goalDirection);
  // return true;
  //if (xxxReturn)
  if (!IS_DEBUG) clearPathAnnotation(sideThreshold, behindThreshold, goalDirection);
  return xxxReturn;
}

// ----------------------------------------------------------------------------

void CtfSeeker::clearPathAnnotation(const float sideThreshold,
  const float behindThreshold,
  const Vec3& goalDirection)
{
  const Vec3 behindSide = side() * sideThreshold;
  const Vec3 behindBack = forward() * -behindThreshold;
  const Vec3 pbb = position() + behindBack;
  const Vec3 gun = localRotateForwardToSide(goalDirection);
  const Vec3 gn = gun * sideThreshold;
  const Vec3 hbc = gHomeBaseCenter;
  if (!IS_DEBUG) annotationLine(pbb + gn, hbc + gn, clearPathColor);
  if (!IS_DEBUG) annotationLine(pbb - gn, hbc - gn, clearPathColor);
  if (!IS_DEBUG) annotationLine(hbc - gn, hbc + gn, clearPathColor);
  if (!IS_DEBUG) annotationLine(pbb - behindSide, pbb + behindSide, clearPathColor);
}


// ----------------------------------------------------------------------------
// xxx perhaps this should be a call to a general purpose annotation
// xxx for "local xxx axis aligned box in XZ plane" -- same code in in
// xxx Pedestrian.cpp


void CtfBase::annotateAvoidObstacle(const float minDistanceToCollision)
{
  const Vec3 boxSide = side() * radius();
  const Vec3 boxFront = forward() * minDistanceToCollision;
  const Vec3 FR = position() + boxFront - boxSide;
  const Vec3 FL = position() + boxFront + boxSide;
  const Vec3 BR = position() - boxSide;
  const Vec3 BL = position() + boxSide;
  const Vec3 white(1, 1, 1);
  if (!IS_DEBUG) annotationLine(FR, FL, white);
  if (!IS_DEBUG) annotationLine(FL, BL, white);
  if (!IS_DEBUG) annotationLine(BL, BR, white);
  if (!IS_DEBUG) annotationLine(BR, FR, white);
}

// ----------------------------------------------------------------------------

Vec3 CtfSeeker::steerToEvadeAllDefenders(void)
{
  Vec3 evade(0, 0, 0);
  const float goalDistance = Vec3::distance(gHomeBaseCenter, position());

  // sum up weighted evasion
  for (int i = 0; i < ctfEnemyCount; i++)
  {
    const CtfEnemy& e = *ctfEnemies[i];
    const Vec3 eOffset = e.position() - position();
    const float eDistance = eOffset.length();

    const float eForwardDistance = forward().dot(eOffset);
    const float behindThreshold = radius() * 2;
    const bool behind = eForwardDistance < behindThreshold;
    if ((!behind) || (eDistance < 5))
    {
      if (eDistance < (goalDistance * 1.2)) //xxx
      {
        // const float timeEstimate = 0.5f * eDistance / e.speed;//xxx
        const float timeEstimate = 0.15f * eDistance / e.speed();//xxx
        const Vec3 future =
          e.predictFuturePosition(timeEstimate);

        annotationXZCircle(e.radius(), future, evadeColor, 20); // xxx

        const Vec3 offset = future - position();
        const Vec3 lateral = offset.perpendicularComponent(forward());
        const float d = lateral.length();
        const float weight = -1000 / (d * d);
        evade += (lateral / d) * weight;
      }
    }
  }
  return evade;
}


Vec3 CtfSeeker::XXXsteerToEvadeAllDefenders(void)
{
  // sum up weighted evasion
  Vec3 evade(0, 0, 0);
  for (int i = 0; i < ctfEnemyCount; i++)
  {
    const CtfEnemy& e = *ctfEnemies[i];
    const Vec3 eOffset = e.position() - position();
    const float eDistance = eOffset.length();

    // xxx maybe this should take into account e's heading? xxx
    const float timeEstimate = 0.5f * eDistance / e.speed(); //xxx
    const Vec3 eFuture = e.predictFuturePosition(timeEstimate);

    // annotation
    annotationXZCircle(e.radius(), eFuture, evadeColor, 20);

    // steering to flee from eFuture (enemy's future position)
    const Vec3 flee = xxxsteerForFlee(eFuture);

    const float eForwardDistance = forward().dot(eOffset);
    const float behindThreshold = radius() * -2;

    const float distanceWeight = 4 / eDistance;
    const float forwardWeight = ((eForwardDistance > behindThreshold) ?
      1.0f : 0.5f);

    const Vec3 adjustedFlee = flee * distanceWeight * forwardWeight;

    evade += adjustedFlee;
  }
  return evade;
}

// ----------------------------------------------------------------------------

Vec3 CtfSeeker::steeringForSeeker(void)
{
  // determine if obstacle avodiance is needed
  Vec3 currentTarget;
  if (gSeeker->state == hiding){
    currentTarget = hide();
  }
  else if (gSeeker->state == running){
    currentTarget = gHomeBaseCenter;
  }
  const bool clearPath = clearPathToGoal(currentTarget);
  adjustObstacleAvoidanceLookAhead(clearPath);
  const Vec3 obstacleAvoidance =
    steerToAvoidObstacles(gAvoidancePredictTime,
    (ObstacleGroup&)allObstacles);

  // saved for annotation
  avoiding = (obstacleAvoidance != Vec3::zero);

  if (avoiding)
  {
    // use pure obstacle avoidance if needed
    return obstacleAvoidance;
  }
  else if (gSeeker->state == hiding){

    const Vec3 seek = SteerLibraryMixin::steerToArrive(hide(), 0.05f);
    return seek;
    //return limitMaxDeviationAngle(seek, 0.707f, forward());
  }
  else
  {
    // otherwise seek home base and perhaps evade defenders
    const Vec3 seek = xxxsteerForSeek(gHomeBaseCenter);
    if (clearPath)
    {
      // we have a clear path (defender-free corridor), use pure seek

      // xxx experiment 9-16-02
      Vec3 s = limitMaxDeviationAngle(seek, 0.707f, forward());

      annotationLine(position(), position() + (s * 0.2f), seekColor);
      return s;
    }
    else
    {
      if (0) // xxx testing new evade code xxx
      {
        // combine seek and (forward facing portion of) evasion
        const Vec3 evade = steerToEvadeAllDefenders();
        const Vec3 steer =
          seek + limitMaxDeviationAngle(evade, 0.5f, forward());

        // annotation: show evasion steering force
        annotationLine(position(), position() + (steer*0.2f), evadeColor);
        return steer;
      }
      else
      {
        const Vec3 evade = XXXsteerToEvadeAllDefenders();
        const Vec3 steer = limitMaxDeviationAngle(seek + evade,
          0.707f, forward());

        annotationLine(position(), position() + seek, gRed);
        annotationLine(position(), position() + evade, gGreen);

        // annotation: show evasion steering force
        annotationLine(position(), position() + (steer*0.2f), evadeColor);
        return steer;
      }
    }
  }
}

Q_Learner::worldState* CtfSeeker::get_world_state(){
  float g_dist, g_angle, e_dist, e_angle, e_facing, hs_dist, hs_angle;
  int g_state = 0;
  Vec3 toHome = gHomeBaseCenter - position();
  Vec3 heading = getForward();
  Vec3 toEnemy = ctfEnemies[0]->position() - position();
  Vec3 e_forward = ctfEnemies[0]->getForward();
  Vec3 toHideSpot = hide() - position();

  if (gSeeker->state == tagged)
    g_state = 1;
  if (gSeeker->state == atGoal)
    g_state = 2;
  g_dist = toHome.length();
  e_dist = toEnemy.length();
  hs_dist = toHideSpot.length();

  g_angle = atan2f(toHome.x, toHome.z) - atan2f(heading.x, heading.z);
  e_facing = atan2f(-toEnemy.x, -toEnemy.z) - atan2f(e_forward.x, e_forward.z);
  e_angle = atan2f(toEnemy.x, toEnemy.z) - atan2f(heading.x, heading.z);
  hs_angle = atan2f(toHideSpot.x, toHideSpot.z) - atan2f(heading.x, heading.z);

  Q_Learner::worldState* ws = new Q_Learner::worldState(g_state, g_dist, g_angle, e_dist, e_angle, e_facing, hs_dist, hs_angle);
  // ws->print();
  return ws;
}

// ----------------------------------------------------------------------------
// adjust obstacle avoidance look ahead time: make it large when we are far
// from the goal and heading directly towards it, make it small otherwise.


void CtfSeeker::adjustObstacleAvoidanceLookAhead(const bool clearPath)
{
  if (clearPath)
  {
    evading = false;
    const float goalDistance = Vec3::distance(gHomeBaseCenter, position());
    const bool headingTowardGoal = isAhead(gHomeBaseCenter, 0.98f);
    const bool isNear = (goalDistance / speed()) < gAvoidancePredictTimeMax;
    const bool useMax = headingTowardGoal && !isNear;
    gAvoidancePredictTime =
      (useMax ? gAvoidancePredictTimeMax : gAvoidancePredictTimeMin);
  }
  else
  {
    evading = true;
    gAvoidancePredictTime = gAvoidancePredictTimeMin;
  }
}

// ----------------------------------------------------------------------------

void CtfSeeker::updateState(const float currentTime)
{
  // if we reach the goal before being tagged, switch to atGoal state
  if (state != tagged)
  {
    const float baseDistance = Vec3::distance(position(), gHomeBaseCenter);
    if (baseDistance < (radius() + gHomeBaseRadius)) state = atGoal;
  }

  // update lastRunningTime (holds off reset time)
  if (state != atGoal && state != tagged)
  {
    lastRunningTime = currentTime;
  }
  else
  {
    const float resetDelay = 4;
    const float resetTime = lastRunningTime + resetDelay;
    if (currentTime > resetTime)
    {
      // xxx a royal hack (should do this internal to CTF):
      OpenSteerDemo::queueDelayedResetPlugInXXX();
    }
  }
}

// ----------------------------------------------------------------------------

Vec3 CtfSeeker::GetHidingPosition(const Vec3 posOb, const float radiusOb, const Vec3 posTarget){
  // distance offset from obsticle
  float BoundaryOffset = this->radius();
  float Dist = BoundaryOffset + radiusOb + 1.0f;

  // calculate the vector from the object to the target
  Vec3 toObject = Vec3(posOb - posTarget);

  // scale it given the objects radius and the seeker radius
  Vec3 hidingSpot = (toObject.normalize() * Dist) + posOb;
  return hidingSpot;
}


Vec3 CtfSeeker::hide(){
  float DistToClosest = 10000.0f;
  Vec3 BestHidingSpot = Vec3(100000.0f, 100000.0f, 10000.0f);
  float radii;
  for (int i = 0; i != allObstacles.size(); ++i)
  {
    radii = allObstacles[i]->radius;
    Vec3 hidingSpot = GetHidingPosition(allObstacles[i]->center, radii, ctfEnemies[0]->position());
    float distToSpot = (hidingSpot - this->position()).length();

    if (distToSpot < DistToClosest){
      DistToClosest = distToSpot;
      BestHidingSpot = hidingSpot;
    }
  }

  if (!IS_DEBUG) annotation3dCircle(0.5f, BestHidingSpot, Vec3(0, 1, 0), Vec3(1, 1, 1), 50);

  return BestHidingSpot;
}

// ----------------------------------------------------------------------------

void CtfSeeker::draw(void)
{
  // first call the draw method in the base class
  CtfBase::draw();

  Vec3 toHome = gHomeBaseCenter - position();
  Vec3 heading = getForward();
  Vec3 toEnemy = ctfEnemies[0]->position() - position();
  Vec3 e_forward = ctfEnemies[0]->getForward();
  Vec3 toHideSpot = hide() - position();

  float g_angle = atan2f(toHome.x, toHome.z) - atan2f(heading.x, heading.z);
  float e_facing = atan2f(-toEnemy.x, -toEnemy.z) - atan2f(e_forward.x, e_forward.z);
  float e_angle = atan2f(toEnemy.x, toEnemy.z) - atan2f(heading.x, heading.z);
  float hs_angle = atan2f(toHideSpot.x, toHideSpot.z) - atan2f(heading.x, heading.z);

  Vec3 ray_end = position() + forward().rotateAboutGlobalY(g_angle) * 3.0f;
  if (!IS_DEBUG) annotationLine(position(), ray_end, Vec3(1.0f, 1.0f, 0.0f));
  ray_end = position() + forward().rotateAboutGlobalY(e_facing) * 3.0f;
  if (!IS_DEBUG) annotationLine(position(), ray_end, Vec3(0.0f, 1.0f, 1.0f));
  ray_end = position() + forward().rotateAboutGlobalY(e_angle) * 3.0f;
  if (!IS_DEBUG) annotationLine(position(), ray_end, Vec3(1.0f, 0.0f, 1.0f));
  ray_end = position() + forward().rotateAboutGlobalY(hs_angle) * 3.0f;
  if (!IS_DEBUG) annotationLine(position(), ray_end, Vec3(1.0f, 0.0f, 0.0f));

  // select string describing current seeker state
  char* seekerStateString = "";
  switch (state)
  {
  case running:
    if (avoiding)
      seekerStateString = "avoid obstacle";
    else if (evading)
      seekerStateString = "seek and evade";
    else
      seekerStateString = "seeking goal";
    break;
  case fleeing: seekerStateString = "Fleeing"; break;
  case hiding: seekerStateString = "Hiding"; break;
  case tagged: seekerStateString = "tagged"; break;
  case atGoal: seekerStateString = "reached goal"; break;
  }
  if (!IS_DEBUG) {
    // annote seeker with its state as text
    const Vec3 textOrigin = position() + Vec3(0, 0.25, 0);
    std::ostringstream annote;
    annote << seekerStateString << std::endl;
    annote << std::setprecision(2) << std::setiosflags(std::ios::fixed)
      << speed() << std::ends;
    draw2dTextAt3dLocation(annote, textOrigin, gWhite);

    // display status in the upper left corner of the window
    std::ostringstream status;
    status << seekerStateString << std::endl;
    status << obstacleCount << " obstacles [F1/F2]" << std::endl;
    status << resetCount << " restarts" << std::ends;
    const float h = drawGetWindowHeight();
    const Vec3 screenLocation(10, h - 50, 0);
    draw2dTextAt2dLocation(status, screenLocation, gGray80);
  }
}


// ----------------------------------------------------------------------------
// update method for goal seeker

void CtfSeeker::update(const float currentTime, const float elapsedTime)
{
  // do behavioral state transitions, as needed
  updateState(currentTime);

  if (currentTime > QL_timer){
    /// Q Learning update loop /// ----------------------------------------------
    /// should only be run every second or so (episode)
    // get world state & save for next loop
    curr_world_state = get_world_state();
    float reward = 0.0f;
    float Q_max = 0.0f;
    // choose action 
    Q_Learner::actions action = Q_agent.get_best_action(curr_world_state);
    // apply Q -Learning alogorithm
    if (prev_world_state != NULL){
      // calculate reward function given previous world state;
      reward = Q_agent.get_reward(prev_world_state, curr_world_state);
      // run nn to obtain best choice from this world state
      Q_max = Q_agent.get_Q_max(curr_world_state);
      // update Q learning nn values
      action = Q_agent.train_ann(prev_world_state, curr_world_state);
    }
    // print out debug info
    // printf("c_time: %f, QL_timer: %f\n", currentTime, QL_timer);
    if (gSeeker->state != atGoal && gSeeker->state != tagged){
      switch (action)
      {
      case Q_Learner::_SEEK:
        //printf("action: SEEK\n\n");
        gSeeker->state = running;
        break;
      case Q_Learner::_EVADE:
        //printf("action: EVADE\n\n");
        gSeeker->state = fleeing;
        break;
      case Q_Learner::_HIDE:
        gSeeker->state = hiding;
        //printf("action: HIDE\n\n");
        break;
      default:
        break;
      }
    }
    //printf("reward: %f\n", reward);
    //printf("max_q: %f\n", Q_max);
    // update QL episode timer
    QL_timer += QL_episode_length;
    prev_world_state = curr_world_state;
  }
  /// Q Learning update loop /// ----------------------------------------------
  // determine and apply steering/braking forces

  Vec3 steer(0, 0, 0);
  switch (gSeeker->state)
  {
  case running:
    steer = xxxsteerForSeek(gHomeBaseCenter);
    break;
  case hiding:
    steer = steerToArrive(hide(), 0.25f);
    break;
  case fleeing:
    steer = steerToEvadeAllDefenders();
  case atGoal:
    applyBrakingForce(gBrakingRate, elapsedTime);
    Q_agent.train_ann(prev_world_state, curr_world_state);
    break;
  case tagged:
    applyBrakingForce(gBrakingRate, elapsedTime);
    Q_agent.train_ann(prev_world_state, curr_world_state);
    break;
  default:
    break;
  }

  // Obstacle Avoidance 
  const bool clearPath = clearPathToGoal(steer);
  adjustObstacleAvoidanceLookAhead(clearPath);
  const Vec3 obstacleAvoidance = steerToAvoidObstacles(gAvoidancePredictTime, (ObstacleGroup&)allObstacles);
  // saved for annotation
  avoiding = (obstacleAvoidance != Vec3::zero);
  // use pure obstacle avoidance if needed
  steer = (avoiding) ? obstacleAvoidance : steer;
  // end obstacle avoidance

  // application of steering force
  applySteeringForce(steer, elapsedTime);

  // annotation
  if (!IS_DEBUG) annotationVelocityAcceleration();
  if (!IS_DEBUG) recordTrailVertex(currentTime, position());
}

// ----------------------------------------------------------------------------
// dynamic obstacle registry
//
// xxx need to combine guts of addOneObstacle and minDistanceToObstacle,
// xxx perhaps by having the former call the latter, or change the latter to
// xxx be "nearestObstacle": give it a position, it finds the nearest obstacle
// xxx (but remember: obstacles a not necessarilty spheres!)


int CtfBase::obstacleCount = -1; // this value means "uninitialized"
SOG CtfBase::allObstacles;


#define testOneObstacleOverlap(radius, center)               \
  {                                                          \
    float d = Vec3::distance (c, center);                    \
    float clearance = d - (r + (radius));                    \
    if (minClearance > clearance) minClearance = clearance;  \
  }


void CtfBase::initializeObstacles(void)
{
  // start with 40% of possible obstacles
  if (obstacleCount == -1)
  {
    obstacleCount = 0;
    for (int i = 0; i < (maxObstacleCount * 0.4); i++) addOneObstacle();
  }
}


void CtfBase::addOneObstacle(void)
{
  if (obstacleCount < maxObstacleCount)
  {
    // pick a random center and radius,
    // loop until no overlap with other obstacles and the home base
    float r;
    Vec3 c;
    float minClearance;
    const float requiredClearance = gSeeker->radius() * 4; // 2 x diameter
    do
    {
      r = frandom2(1.5, 4);
      c = randomVectorOnUnitRadiusXZDisk() * gMaxStartRadius * 1.1f;
      minClearance = FLT_MAX;

      for (SOI so = allObstacles.begin(); so != allObstacles.end(); so++)
      {
        testOneObstacleOverlap((**so).radius, (**so).center);
      }

      testOneObstacleOverlap(gHomeBaseRadius - requiredClearance,
        gHomeBaseCenter);
    } while (minClearance < requiredClearance);

    // add new non-overlapping obstacle to registry
    allObstacles.push_back(new SphericalObstacle(r, c));
    obstacleCount++;
  }
}


float CtfBase::minDistanceToObstacle(const Vec3 point)
{
  float r = 0;
  Vec3 c = point;
  float minClearance = FLT_MAX;
  for (SOI so = allObstacles.begin(); so != allObstacles.end(); so++)
  {
    testOneObstacleOverlap((**so).radius, (**so).center);
  }
  return minClearance;
}


void CtfBase::removeOneObstacle(void)
{
  if (obstacleCount > 0)
  {
    obstacleCount--;
    allObstacles.pop_back();
  }
}


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class CtfPlugIn : public PlugIn
{
public:

  const char* name(void) { return "Capture the Flag"; }

  float selectionOrderSortKey(void) { return 0.01f; }

  virtual ~CtfPlugIn() {} // be more "nice" to avoid a compiler warning

  void open(void)
  {
    // create the seeker ("hero"/"attacker")
    ctfSeeker = new CtfSeeker;
    all.push_back(ctfSeeker);

    // create the specified number of enemies, 
    // storing pointers to them in an array.
    for (int i = 0; i < ctfEnemyCount; i++)
    {
      ctfEnemies[i] = new CtfEnemy;
      all.push_back(ctfEnemies[i]);
    }

    // initialize camera
    OpenSteerDemo::init2dCamera(*ctfSeeker);
    OpenSteerDemo::camera.mode = Camera::cmFixedDistanceOffset;
    OpenSteerDemo::camera.fixedTarget.set(15, 0, 0);
    OpenSteerDemo::camera.fixedPosition.set(80, 60, 0);

    CtfBase::initializeObstacles();

  }

  void update(const float currentTime, const float elapsedTime)
  {
    // update the seeker
    ctfSeeker->update(currentTime, elapsedTime);

    // update each enemy
    for (int i = 0; i < ctfEnemyCount; i++)
    {
      ctfEnemies[i]->update(currentTime, elapsedTime);
    }
  }

  void redraw(const float currentTime, const float elapsedTime)
  {
    // selected vehicle (user can mouse click to select another)
    AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

    // vehicle nearest mouse (to be highlighted)
    AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse();

    // update camera
    OpenSteerDemo::updateCamera(currentTime, elapsedTime, selected);

    // draw "ground plane" centered between base and selected vehicle
    const Vec3 goalOffset = gHomeBaseCenter - OpenSteerDemo::camera.position();
    const Vec3 goalDirection = goalOffset.normalize();
    const Vec3 cameraForward = OpenSteerDemo::camera.xxxls().forward();
    const float goalDot = cameraForward.dot(goalDirection);
    const float blend = remapIntervalClip(goalDot, 1, 0, 0.5, 0);
    const Vec3 gridCenter = interpolate(blend,
      selected.position(),
      gHomeBaseCenter);
    OpenSteerDemo::gridUtility(gridCenter);

    // draw the seeker, obstacles and home base
    ctfSeeker->draw();
    drawObstacles();
    drawHomeBase();

    // draw each enemy
    for (int i = 0; i < ctfEnemyCount; i++) ctfEnemies[i]->draw();

    // highlight vehicle nearest mouse
    OpenSteerDemo::highlightVehicleUtility(nearMouse);
  }

  void close(void)
  {
    // delete seeker
    delete (ctfSeeker);
    ctfSeeker = NULL;

    // delete each enemy
    for (int i = 0; i < ctfEnemyCount; i++)
    {
      delete (ctfEnemies[i]);
      ctfEnemies[i] = NULL;
    }

    // clear the group of all vehicles
    all.clear();
  }

  void reset(void)
  {
    // count resets
    resetCount++;

    // reset the seeker ("hero"/"attacker") and enemies
    ctfSeeker->reset();
    for (int i = 0; i < ctfEnemyCount; i++) ctfEnemies[i]->reset();

    // reset camera position
    OpenSteerDemo::position2dCamera(*ctfSeeker);

    // make camera jump immediately to new position
    OpenSteerDemo::camera.doNotSmoothNextMove();
  }

  void handleFunctionKeys(int keyNumber)
  {
    switch (keyNumber)
    {
    case 1: CtfBase::addOneObstacle();    break;
    case 2: CtfBase::removeOneObstacle(); break;
    }
  }

  void printMiniHelpForFunctionKeys(void)
  {
    std::ostringstream message;
    message << "Function keys handled by ";
    message << '"' << name() << '"' << ':' << std::ends;
    OpenSteerDemo::printMessage(message);
    OpenSteerDemo::printMessage("  F1     add one obstacle.");
    OpenSteerDemo::printMessage("  F2     remove one obstacle.");
    OpenSteerDemo::printMessage("");
  }

  const AVGroup& allVehicles(void) { return (const AVGroup&)all; }

  void drawHomeBase(void)
  {
    const Vec3 up(0, 0.01f, 0);
    const Vec3 atColor(0.3f, 0.3f, 0.5f);
    const Vec3 noColor = gGray50;
    const bool reached = ctfSeeker->state == CtfSeeker::atGoal;
    const Vec3 baseColor = (reached ? atColor : noColor);
    drawXZDisk(gHomeBaseRadius, gHomeBaseCenter, baseColor, 40);
    drawXZDisk(gHomeBaseRadius / 15, gHomeBaseCenter + up, gBlack, 20);
  }

  void drawObstacles(void)
  {
    const Vec3 color(0.8f, 0.6f, 0.4f);
    const SOG& allSO = CtfBase::allObstacles;
    for (SOI so = allSO.begin(); so != allSO.end(); so++)
    {
      drawXZCircle((**so).radius, (**so).center, color, 40);
    }
  }

  // a group (STL vector) of all vehicles in the PlugIn
  std::vector<CtfBase*> all;
};


CtfPlugIn gCtfPlugIn;


// ----------------------------------------------------------------------------
