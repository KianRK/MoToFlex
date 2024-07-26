/**
 * @file SoundRequest.h
 * Declaration of class SoundRequest.
 * @author Philippe Schober
 */ 

#ifndef __SoundRequest_h_
#define __SoundRequest_h_

#include "Tools/Streams/Streamable.h"

class SoundRequest : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
      STREAM_ENUM(sound, numOfSounds, SoundRequest::getSoundName);
      STREAM(once);
    STREAM_REGISTER_FINISH();
  }

public:
  enum Sound
  {
    none,
		adjusting_for_kick,
		amazing,
		approachKick,
		battery_5,
		battery_15,
		battery_30,
		battery_low,
		beback,
		blue,
		boring,
		challenger,
		connected,
		correcting,
		defender,
		dot,
		eight,
		femshutdown,
		femstartup,
		finished,
		fire,
		five,
		four,
		goingBackGoal,
		goToBall,
		hasta,
		heat,
		i_am_goalie,
		i_am_penalized,
		incoming,
		say_initial,
		kick_done,
		lowbattery,
		lower,
		nine,
		no_network,
		noBall,
		notPenalized,
		one,
		opponent_kickoff,
		own_kickoff,
		penalized,
		playing,
		playing_kickoff_now,
		playingDefensiveGoalie,
		position,
		ready,
		red,
		reload,
		roadrunner,
		runaway,
		seeBall,
		set,
		seven,
		six,
		sm64_game_over,
		sm64_here_we_go,
		sm64_high_score,
		sm64_injury,
		sm64_lets_go,
		starting_option_1,
		starting_option_2,
		starting_option_3,
		starting_option_4,
		starting_option_5,
		stop,
		striker,
		three,
		two,
		upper,
		wireless_turned_off,
		wireless_turned_on,
		yellow,
		zero,
    numOfSounds
  };

  Sound sound; /**< The requested sound to be played. */
  bool once; /**< Play sound only once. */
 
  static const char* getSoundName(Sound sound)
  {
    switch(sound)
    {
			case none: return "none";
			case adjusting_for_kick: return "adjusting_for_kick";
			case amazing: return "amazing";
			case approachKick: return "approachKick";
			case battery_5: return "battery_5";
			case battery_15: return "battery_15";
			case battery_30: return "battery_30";
			case battery_low: return "battery_low";
			case beback: return "beback";
			case blue: return "blue";
			case boring: return "boring";
			case challenger: return "challenger";
			case connected: return "connected";
			case correcting: return "correcting";
			case defender: return "defender";
			case dot: return "dot";
			case eight: return "eight";
			case femshutdown: return "femshutdown";
			case femstartup: return "femstartup";
			case finished: return "finished";
			case fire: return "fire";
			case five: return "five";
			case four: return "four";
			case goingBackGoal: return "goingBackGoal";
			case goToBall: return "goToBall";
			case hasta: return "hasta";
			case heat: return "heat";
			case i_am_goalie: return "i_am_goalie";
			case i_am_penalized: return "i_am_penalized";
			case incoming: return "incoming";
			case say_initial: return "say_initial";
			case kick_done: return "kick_done";
			case lowbattery: return "lowbattery";
			case lower: return "lower";
			case nine: return "nine";
			case no_network: return "no_network";
			case noBall: return "noBall";
			case notPenalized: return "notPenalized";
			case one: return "one";
			case opponent_kickoff: return "opponent_kickoff";
			case own_kickoff: return "own_kickoff";
			case penalized: return "penalized";
			case playing: return "playing";
			case playing_kickoff_now: return "playing_kickoff_now";
			case playingDefensiveGoalie: return "playingDefensiveGoalie";
			case position: return "position";
			case ready: return "ready";
			case red: return "red";
			case reload: return "reload";
			case roadrunner: return "roadrunner";
			case runaway: return "runaway";
			case seeBall: return "seeBall";
			case set: return "set";
			case seven: return "seven";
			case six: return "six";
			case sm64_game_over: return "sm64_game_over";
			case sm64_here_we_go: return "sm64_here_we_go";
			case sm64_high_score: return "sm64_high_score";
			case sm64_injury: return "sm64_injury";
			case sm64_lets_go: return "sm64_lets_go";
			case starting_option_1: return "starting_option_1";
			case starting_option_2: return "starting_option_2";
			case starting_option_3: return "starting_option_3";
			case starting_option_4: return "starting_option_4";
			case starting_option_5: return "starting_option_5";
			case stop: return "stop";
			case striker: return "striker";
			case three: return "three";
			case two: return "two";
			case upper: return "upper";
			case wireless_turned_off: return "wireless_turned_off";
			case wireless_turned_on: return "wireless_turned_on";
			case yellow: return "yellow";
			case zero: return "zero";
      default: return "unknown";
    }
  };

  /**
  * The function returns the filename of the sound to play.
  * @param sound The sound.
  * @return The filename of this sound.
  */
  static const char* getSoundFilename(Sound sound)
  {
    switch(sound)
    {
		case none: return "none.wav";
		case adjusting_for_kick: return "adjusting_for_kick.wav";
		case amazing: return "amazing.wav";
		case approachKick: return "approachKick.wav";
		case battery_5: return "battery_5.wav";
		case battery_15: return "battery_15.wav";
		case battery_30: return "battery_30.wav";
		case battery_low: return "battery_low.wav";
		case beback: return "beback.wav";
		case blue: return "blue.wav";
		case boring: return "boring.wav";
		case challenger: return "challenger.wav";
		case connected: return "connected.wav";
		case correcting: return "correcting.wav";
		case defender: return "defender.wav";
		case dot: return "dot.wav";
		case eight: return "eight.wav";
		case femshutdown: return "femshutdown.wav";
		case femstartup: return "femstartup.wav";
		case finished: return "finished.wav";
		case fire: return "fire.wav";
		case five: return "five.wav";
		case four: return "four.wav";
		case goingBackGoal: return "goingBackGoal.wav";
		case goToBall: return "goToBall.wav";
		case hasta: return "hasta.wav";
		case heat: return "heat.wav";
		case i_am_goalie: return "i_am_goalie.wav";
		case i_am_penalized: return "i_am_penalized.wav";
		case incoming: return "incoming.wav";
		case say_initial: return "initial.wav";
		case kick_done: return "kick_done.wav";
		case lowbattery: return "lowbattery.wav";
		case lower: return "lower.wav";
		case nine: return "nine.wav";
		case no_network: return "no_network.wav";
		case noBall: return "noBall.wav";
		case notPenalized: return "notPenalized.wav";
		case one: return "one.wav";
		case opponent_kickoff: return "opponent_kickoff.wav";
		case own_kickoff: return "own_kickoff.wav";
		case penalized: return "penalized.wav";
		case playing: return "playing.wav";
		case playing_kickoff_now: return "playing_kickoff_now.wav";
		case playingDefensiveGoalie: return "playingDefensiveGoalie.wav";
		case position: return "position.wav";
		case ready: return "ready.wav";
		case red: return "red.wav";
		case reload: return "reload.wav";
		case roadrunner: return "roadrunner.wav";
		case runaway: return "runaway.wav";
		case seeBall: return "seeBall.wav";
		case set: return "set.wav";
		case seven: return "set.wav";
		case six: return "six.wav";
		case sm64_game_over: return "sm64_game_over.wav";
		case sm64_here_we_go: return "sm64_here_we_go.wav";
		case sm64_high_score: return "sm64_high_score.wav";
		case sm64_injury: return "sm64_injury.wav";
		case sm64_lets_go: return "sm64_lets_go.wav";
		case starting_option_1: return "starting_option_1.wav";
		case starting_option_2: return "starting_option_2.wav";
		case starting_option_3: return "starting_option_3.wav";
		case starting_option_4: return "starting_option_4.wav";
		case starting_option_5: return "starting_option_5.wav";
		case stop: return "stop.wav";
		case striker: return "striker.wav";
		case three: return "three.wav";
		case two: return "two.wav";
		case upper: return "upper.wav";
		case wireless_turned_off: return "wireless_turned_off.wav";
		case wireless_turned_on: return "wireless_turned_on.wav";
		case yellow: return "yellow.wav";
		case zero: return "zero.wav";
    default: return 0;
    }
  };

  /**
  * Default constructor.
  */
  SoundRequest() : sound(none), once(true) {}
};

/**
* A dummy class so SoundControl is selectable.
*/
class SoundOutput : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
      char dummy(0);
      STREAM(dummy);
    STREAM_REGISTER_FINISH();
  }
};

#endif //__SoundRequest_h_
