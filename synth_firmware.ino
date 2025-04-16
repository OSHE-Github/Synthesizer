#include <MIDI.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI); // create MIDI instance listening on Serial1 

const int voices = 8; // how many notes can be played at once
const int oscillators = 2; // how many oscillators (waveforms) per voice
const int waveforms = 5; // how many different waveshapes to choose from
const int waveform_switchA1 = 32; // pins of waveform selector switch for oscillators A and B
const int waveform_switchB1 = 30; 
const int waveform_switchA2 = 31;
const int waveform_switchB2 = 29;
const int mux1_s0 = 35; // pins used for mux select bits
const int mux1_s1 = 36;
const int mux1_s2 = 37;
const int mux1_inputs = 8; // how many inputs are used on mux 1

bool found_match = false; // utility variable used for searching arrays
byte midi_type = 0; // type of MIDI message being sent
int *midi_stack_ptr; // address of midi stack array
int default_waveform = 0;
int midi_note = 0; // which note was played
int midi_stack_index = 0; // current location in the MIDI stack
int notes_pressed_count = 0; // how many notes are currently held down
int notes_on_count = 0; // how many notes are currently sounding
int analog_res = 1024; // bit resolution used by ADCs
int detune_pot = 0; // _pot are values read from respective pots
int cutoff_pot = 0; 
int resonance_pot = 0; 
int amp_envelope_attack_pot = 0;
int amp_envelope_decay_pot = 0;
int amp_envelope_sustain_pot = 0;
int amp_envelope_release_pot = 0;
int pulse_width_pot = 0;
int volume_pot = 0;
int osc_mix_pot = 0;
int cutoff_range = 14950; // frequency range of filter (Hz)
int envelope_time_range = 11880; // time range of envelope parameters (ms)
int release_queue_count = 0; // amount of voices currently releasing
int waveform_switch_stateA = 0; // current status of the encoder (being turned or not)
int prev_waveform_switch_stateA = 1; // previous status of the encoder
int waveform_switch_stateB = 0;
int prev_waveform_switch_stateB = 1;
int new_voice = 0; // voice chosen to be used by allocation logic
float max_voice_gain = 0.12; // maximum allowed gain of a single voice (1/8 because 8 voices)
float filter_cutoff = 0; // current filter cutoff (Hz)
float filter_resonance = 0; // current filter resonance gain
float osc2_detune = 0; // current oscillator 2 detune (percent)
float pulse_width = 0; // current pulse width of pulse waveform (percent / 100)
float amp_envelope_decay = 0; // current amp envelope decay time (ms)
float amp_envelope_attack = 0; // current amp envelope attack time (ms)
float amp_envelope_release = 0; // currnet amp envelope release time (ms)
float pulse_width_range = 0.47; // range of pulse waveform pulse width (percent / 100)
float pulse_width_inc = 0; // _inc are scalar multipliers for interpreting pot values
float output_volume = 0; // gain of output volume (0-1)
float envelope_time_inc = 0;
float amp_sustain_level = 0; // envelope sustain level (0-1)
float cutoff_inc = 0;
float resonance_range = 4.3; // range of filter resonance (gain)
float resonance_inc = 0;
float frequency = 0; // frequency of note
float detune_range = 0.02973154717; // range of detuning oscillator 2 (percent - 1)
float detune_inc = 0;
float osc_mix_inc = 0;
float osc1_gain = 0; // gain level for oscillator 1
float osc2_gain = 0; //gain level for oscillator 2
bool voices_on[voices] = {}; // which voices are currently being used by a held down note
bool notes_pressed[128]; // which notes are currently being pressed
bool notes_on[128]; // which notes are currently sounding
int mux_decode[8][3] = {{LOW, LOW, LOW}, {HIGH, LOW, LOW}, {LOW, HIGH, LOW}, {HIGH, HIGH, LOW}, {LOW, LOW, HIGH}, {HIGH, LOW, HIGH}, {LOW, HIGH, HIGH}, {HIGH, HIGH, HIGH}}; // maps mux input number to select signals
int midi_stack[128]; // LIFO of notes pressed
int voice_notes[voices] = {}; // which note is currently assigned to each voice
int release_queue[voices] = {}; // FIFO of voices still in release stage
int osc_waveforms[oscillators]; // which waveform is currently assigned to each oscillator
int waveform_lookup[waveforms] = {1, 2, 5, 3, 0}; // which waveforms are available (saw, square, pulse, triangle, sine)
float freq_table[128]; // lookup table for which frequency is for which MIDI note
float voice_freqs[voices]; // base note frequency currently assigned to each voice

AudioSynthWaveform osc1A; // creating all 16 oscillators (2 per voice)
AudioSynthWaveform osc1B;
AudioSynthWaveform osc2A;
AudioSynthWaveform osc2B;
AudioSynthWaveform osc3A;
AudioSynthWaveform osc3B;
AudioSynthWaveform osc4A;
AudioSynthWaveform osc4B;
AudioSynthWaveform osc5A;
AudioSynthWaveform osc5B;
AudioSynthWaveform osc6A;
AudioSynthWaveform osc6B;
AudioSynthWaveform osc7A;
AudioSynthWaveform osc7B;
AudioSynthWaveform osc8A;
AudioSynthWaveform osc8B;
AudioFilterStateVariable filter1; // creating all 8 filters
AudioFilterStateVariable filter2;
AudioFilterStateVariable filter3;
AudioFilterStateVariable filter4;
AudioFilterStateVariable filter5;
AudioFilterStateVariable filter6;
AudioFilterStateVariable filter7;
AudioFilterStateVariable filter8;
AudioEffectEnvelope amp_envelope1; // creating all 8 amplitude envelopes
AudioEffectEnvelope amp_envelope2;
AudioEffectEnvelope amp_envelope3;
AudioEffectEnvelope amp_envelope4;
AudioEffectEnvelope amp_envelope5;
AudioEffectEnvelope amp_envelope6;
AudioEffectEnvelope amp_envelope7;
AudioEffectEnvelope amp_envelope8;
AudioMixer4 voice1mix; // creating mixers used to create signal path
AudioMixer4 voice2mix;
AudioMixer4 voice3mix;
AudioMixer4 voice4mix;
AudioMixer4 voice5mix;
AudioMixer4 voice6mix;
AudioMixer4 voice7mix;
AudioMixer4 voice8mix;
AudioMixer4 voices1;
AudioMixer4 voices2;
AudioMixer4 voices3;
AudioOutputI2S i2s_out; // stereo output from audio shield DAC

AudioConnection patch1(osc1A, 0, voice1mix, 0); // connections to create signal path (Refer to DSP block diagram)
AudioConnection patch2(osc1B, 0, voice1mix, 1);
AudioConnection patch3(osc2A, 0, voice2mix, 0);
AudioConnection patch4(osc2B, 0, voice2mix, 1);
AudioConnection patch5(osc3A, 0, voice3mix, 0);
AudioConnection patch6(osc3B, 0, voice3mix, 1);
AudioConnection patch7(osc4A, 0, voice4mix, 0);
AudioConnection patch8(osc4B, 0, voice4mix, 1);
AudioConnection patch9(osc5A, 0, voice5mix, 0);
AudioConnection patch10(osc5B, 0, voice5mix, 1);
AudioConnection patch11(osc6A, 0, voice6mix, 0);
AudioConnection patch12(osc6B, 0, voice6mix, 1);
AudioConnection patch13(osc7A, 0, voice7mix, 0);
AudioConnection patch14(osc7B, 0, voice7mix, 1);
AudioConnection patch15(osc8A, 0, voice8mix, 0);
AudioConnection patch16(osc8B, 0, voice8mix, 1);
AudioConnection patch29(voice1mix, 0, filter1, 0);
AudioConnection patch30(voice2mix, 0, filter2, 0);
AudioConnection patch31(voice3mix, 0, filter3, 0);
AudioConnection patch32(voice4mix, 0, filter4, 0);
AudioConnection patch33(voice5mix, 0, filter5, 0);
AudioConnection patch34(voice6mix, 0, filter6, 0);
AudioConnection patch35(voice7mix, 0, filter7, 0);
AudioConnection patch36(voice8mix, 0, filter8, 0);
AudioConnection patch37(filter1, 0, amp_envelope1, 0);
AudioConnection patch38(filter2, 0, amp_envelope2, 0);
AudioConnection patch39(filter3, 0, amp_envelope3, 0);
AudioConnection patch40(filter4, 0, amp_envelope4, 0);
AudioConnection patch41(filter5, 0, amp_envelope5, 0);
AudioConnection patch42(filter6, 0, amp_envelope6, 0);
AudioConnection patch43(filter7, 0, amp_envelope7, 0);
AudioConnection patch44(filter8, 0, amp_envelope8, 0);
AudioConnection patch17(amp_envelope1, 0, voices1, 0);
AudioConnection patch18(amp_envelope2, 0, voices1, 1);
AudioConnection patch19(amp_envelope3, 0, voices1, 2);
AudioConnection patch20(amp_envelope4, 0, voices1, 3);
AudioConnection patch21(amp_envelope5, 0, voices2, 0);
AudioConnection patch22(amp_envelope6, 0, voices2, 1);
AudioConnection patch23(amp_envelope7, 0, voices2, 2);
AudioConnection patch24(amp_envelope8, 0, voices2, 3);
AudioConnection patch25(voices1, 0, voices3, 0);
AudioConnection patch26(voices2, 0, voices3, 1);
AudioConnection patch27(voices3, 0, i2s_out, 0);
AudioConnection patch28(voices3, 0, i2s_out, 1);

AudioControlSGTL5000 sgtl5000; // creating audio controller

void setup(){ // code that runs once at bootup
  MIDI.begin(MIDI_CHANNEL_OMNI); // start listening for MIDI messages
  Serial.begin(57600); // correct baud rate for MIDI I guess?
  AudioMemory(40); // allocate a bunch of memory for the audio library
  sgtl5000.enable(); // turn on audio controller
  sgtl5000.volume(0.51); // headphone volume
  voice1mix.gain(0, 0.06); // set all oscillators to initially have a gain of 1/16 (because 16 oscillators) to avoid distortion
  voice1mix.gain(1, 0.06);
  voice2mix.gain(0, 0.06);
  voice2mix.gain(1, 0.06);
  voice3mix.gain(0, 0.06);
  voice3mix.gain(1, 0.06);
  voice4mix.gain(0, 0.06);
  voice4mix.gain(1, 0.06);
  voice5mix.gain(0, 0.06);
  voice5mix.gain(1, 0.06);
  voice6mix.gain(0, 0.06);
  voice6mix.gain(1, 0.06);
  voice7mix.gain(0, 0.06);
  voice7mix.gain(1, 0.06);
  voice8mix.gain(0, 0.06);
  voice8mix.gain(1, 0.06); 
  voices1.gain(0, 1); // all following gains can stay at 1
  voices1.gain(1, 1);
  voices1.gain(2, 1);
  voices1.gain(3, 1);
  voices2.gain(0, 1);
  voices2.gain(1, 1);
  voices2.gain(2, 1);
  voices2.gain(3, 1);
  voices3.gain(0, 1);
  voices3.gain(1, 1);
  pinMode(32, INPUT); // waveform select pins should be inputs
  pinMode(31, INPUT); 
  pinMode(30, INPUT);
  pinMode(29, INPUT);
  pinMode(35, OUTPUT); //  mux select pins should be output
  pinMode(36, OUTPUT);
  pinMode(37, OUTPUT);
  midi_stack_ptr = &midi_stack[0]; // address of MIDI stack
  detune_inc = detune_range / analog_res; // calculate increase percentage per increment of pot value
  cutoff_inc = cutoff_range / analog_res; // calculate increase of cutoff frequency (Hz) per increment of pot value
  resonance_inc = resonance_range / analog_res; // calculate increase of resonange gain per increment of pot value
  envelope_time_inc = envelope_time_range / analog_res; // calculate increase of envelope time (ms) per increment of pot value
  pulse_width_inc = pulse_width_range / analog_res; // calculate increase of pulse width (percent / 100) per increment of pot value
  osc_mix_inc = max_voice_gain / analog_res; // calculate increase of oscillator gain per increment of pot value
  create_freq_table(); // fill frequency table that associates each MIDI note with its frequency
  osc1A.begin(waveform_lookup[default_waveform]); // each oscillator should first boot up with whatever the default waveform is set to
  osc1B.begin(waveform_lookup[default_waveform]);
  osc2A.begin(waveform_lookup[default_waveform]);
  osc2B.begin(waveform_lookup[default_waveform]);
  osc3A.begin(waveform_lookup[default_waveform]);
  osc3B.begin(waveform_lookup[default_waveform]);
  osc4A.begin(waveform_lookup[default_waveform]);
  osc4B.begin(waveform_lookup[default_waveform]);
  osc5A.begin(waveform_lookup[default_waveform]);
  osc5B.begin(waveform_lookup[default_waveform]);
  osc6A.begin(waveform_lookup[default_waveform]);
  osc6B.begin(waveform_lookup[default_waveform]);
  osc7A.begin(waveform_lookup[default_waveform]);
  osc7B.begin(waveform_lookup[default_waveform]);
  osc8A.begin(waveform_lookup[default_waveform]);
  osc8B.begin(waveform_lookup[default_waveform]);
}

void create_freq_table(){ // fills frequency table that associates each MIDI note with its frequency
  for(int i = 0; i < 128; i++){
    float curr_freq = i - 69; // equation for converting MIDI note to note frequency
    curr_freq = curr_freq / 12;
    curr_freq = pow(2, curr_freq);
    curr_freq = 440 * curr_freq;
    freq_table[i] = curr_freq;
  }
}

bool check_amp_envelope(int voice){ // checks the amp envelope of a given voice to see if it is currently still in any stage of ADSR
  switch(voice){
    case 0:
      if(amp_envelope1.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 1:
      if(amp_envelope2.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 2:
      if(amp_envelope3.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 3:
      if(amp_envelope4.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 4:
      if(amp_envelope5.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 5:
      if(amp_envelope6.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 6:
      if(amp_envelope7.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    case 7:
      if(amp_envelope8.isActive()){
        return true;
      }else{
        return false;
      }
      break;
    default:
      return false;
  }
}

void update_amp_envelope(int voice, bool start){ // begins (starts attack phase) or ends (begins release phase) the amp envelope of a given voice
  switch(voice){
    case 0:
      if(start){ // if envelope should be starting
        amp_envelope1.noteOn();
        osc1A.amplitude(0.3); // kind of arbitrary maximum oscillator amplitude
        osc1B.amplitude(0.3);
      }else{ // if envelope should be ending
        amp_envelope1.noteOff();
      }
      break;
    case 1:
      if(start){
        amp_envelope2.noteOn();
        osc2A.amplitude(0.3);
        osc2B.amplitude(0.3);
      }else{
        amp_envelope2.noteOff();
      }
      break;
    case 2:
      if(start){
        amp_envelope3.noteOn();
        osc3A.amplitude(0.3);
        osc3B.amplitude(0.3);
      }else{
        amp_envelope3.noteOff();
      }
      break;
    case 3:
      if(start){
        amp_envelope4.noteOn();
        osc4A.amplitude(0.3);
        osc4B.amplitude(0.3);
      }else{
        amp_envelope4.noteOff();
      }
      break;
    case 4:
      if(start){
        amp_envelope5.noteOn();
        osc5A.amplitude(0.3);
        osc5B.amplitude(0.3);
      }else{
        amp_envelope5.noteOff();
      }
      break;
    case 5:
      if(start){
        amp_envelope6.noteOn();
        osc6A.amplitude(0.3);
        osc6B.amplitude(0.3);
      }else{
        amp_envelope6.noteOff();
      }
      break;
    case 6:
      if(start){
        amp_envelope7.noteOn();
        osc7A.amplitude(0.3);
        osc7B.amplitude(0.3);
      }else{
        amp_envelope7.noteOff();
      }
      break;
    case 7:
      if(start){
        amp_envelope8.noteOn();
        osc8A.amplitude(0.3);
        osc8B.amplitude(0.3);
      }else{
        amp_envelope8.noteOff();
      }
      break;
  }
}

void update_note(int voice){ // changes the note (frequency) that the oscillators of a given voice are playing
  switch(voice){
    case 0:
      osc1A.frequency(voice_freqs[0]);
      osc1B.frequency(voice_freqs[0] * osc2_detune); // oscillator 2 can be detuned from oscillator 1
      break;
    case 1:
      osc2A.frequency(voice_freqs[1]);
      osc2B.frequency(voice_freqs[1] * osc2_detune);
      break;
    case 2:
      osc3A.frequency(voice_freqs[2]);
      osc3B.frequency(voice_freqs[2] * osc2_detune);
      break;
    case 3:
      osc4A.frequency(voice_freqs[3]);
      osc4B.frequency(voice_freqs[3] * osc2_detune);
      break;
    case 4:
      osc5A.frequency(voice_freqs[4]);
      osc5B.frequency(voice_freqs[4] * osc2_detune);
      break;
    case 5:
      osc6A.frequency(voice_freqs[5]);
      osc6B.frequency(voice_freqs[5] * osc2_detune);
      break;
    case 6:
      osc7A.frequency(voice_freqs[6]);
      osc7B.frequency(voice_freqs[6] * osc2_detune);
      break;
    case 7:
      osc8A.frequency(voice_freqs[7]);
      osc8B.frequency(voice_freqs[7] * osc2_detune);
      break;
  }
}

void loop(){ // code that actually runs over and over
  for(int i = 0; i < mux1_inputs; i++){ // for each analog input into the mux
    digitalWrite(mux1_s0, mux_decode[i][0]); // set the select bits on mux to connect the correct input to the ADC
    digitalWrite(mux1_s1, mux_decode[i][1]);
    digitalWrite(mux1_s2, mux_decode[i][2]);

    switch(i){ // when a certain mux input is active, update its respective pot reading
      case 0:
        amp_envelope_sustain_pot = analogRead(A14);
        break;
      case 1:
        amp_envelope_decay_pot = analogRead(A14);
        break;
      case 2:
        amp_envelope_attack_pot = analogRead(A14);
        break;
      case 3:
        amp_envelope_release_pot = analogRead(A14);
        break;
      case 4:
        volume_pot = analogRead(A14);
        break;
      case 5:
        resonance_pot = analogRead(A14);
        break;
      case 6:
        osc_mix_pot = analogRead(A14);
        break;
      case 7:
        pulse_width_pot = analogRead(A14);
        break;
    }
  }
  cutoff_pot = analogRead(A16); // update all other analog inputs not connected to mux
  detune_pot = analogRead(A17);
  amp_sustain_level = (((float)amp_envelope_sustain_pot) / analog_res); // calcuate amp envelope sustain level (0 - 1)
  output_volume = (((float)volume_pot) / analog_res); // calculate headphone volume level (0 - 1)
  sgtl5000.volume(output_volume); // update headphone volume
  if(osc_mix_pot <= 4){ // in order to make sure one oscillator can be consistently completely off, any ADC reading 4 or below will be read as 0
    osc1_gain = 0;
  }else{
    osc1_gain = osc_mix_pot * osc_mix_inc; // calculate oscillator 1 gain (0 - 0.12)
  }
  osc2_gain = max_voice_gain - osc1_gain; // calculate oscillator 2 gain (0 - 0.12)
  voice1mix.gain(0, osc1_gain); // adjust the mix for the oscillators (gain for oscillators 1 and 2 always add up to 0.12)
  voice1mix.gain(1, osc2_gain);
  voice2mix.gain(0, osc1_gain);
  voice2mix.gain(1, osc2_gain);
  voice3mix.gain(0, osc1_gain);
  voice3mix.gain(1, osc2_gain);
  voice4mix.gain(0, osc1_gain);
  voice4mix.gain(1, osc2_gain);
  voice5mix.gain(0, osc1_gain);
  voice5mix.gain(1, osc2_gain);
  voice6mix.gain(0, osc1_gain);
  voice6mix.gain(1, osc2_gain);
  voice7mix.gain(0, osc1_gain);
  voice7mix.gain(1, osc2_gain);
  voice8mix.gain(0, osc1_gain);
  voice8mix.gain(1, osc2_gain);
  pulse_width = (pulse_width_pot * pulse_width_inc) + 0.03; // calculate pulse width (3% - 50%)
  osc1A.pulseWidth(pulse_width); 
  osc1B.pulseWidth(pulse_width);
  osc2A.pulseWidth(pulse_width);
  osc2B.pulseWidth(pulse_width);
  osc3A.pulseWidth(pulse_width);
  osc3B.pulseWidth(pulse_width);
  osc4A.pulseWidth(pulse_width);
  osc4B.pulseWidth(pulse_width);
  osc5A.pulseWidth(pulse_width);
  osc5B.pulseWidth(pulse_width);
  osc6A.pulseWidth(pulse_width);
  osc6B.pulseWidth(pulse_width);
  osc7A.pulseWidth(pulse_width);
  osc7B.pulseWidth(pulse_width);
  osc8A.pulseWidth(pulse_width);
  osc8B.pulseWidth(pulse_width);
  amp_envelope_attack = amp_envelope_attack_pot * envelope_time_inc; // calculate amp envelope attack (0 - 11880 ms)
  if(amp_envelope_attack < 40){ // in order to get snappy attacks when pot is at minimum, all attacks under 40 ms should be 0 ms
    amp_envelope_attack = 0;
  }
  amp_envelope_decay = amp_envelope_decay_pot * envelope_time_inc; // calculate amp envelope decay (0 - 11880 ms)
  amp_envelope_release = amp_envelope_release_pot * envelope_time_inc; // calculate amp envelope release (0 - 11880 ms)
  if(amp_envelope_release < 40){
    amp_envelope_release = 0; // in order to get snappy releases when pot is at minimum, all releases under 40 ms should be 0 ms
  }
  amp_envelope1.attack(amp_envelope_attack); // update ADSR parameters for all amplitude envelopes
  amp_envelope1.decay(amp_envelope_decay);
  amp_envelope1.sustain(amp_sustain_level);
  amp_envelope1.release(amp_envelope_release);
  amp_envelope2.attack(amp_envelope_attack);
  amp_envelope2.decay(amp_envelope_decay);
  amp_envelope2.sustain(amp_sustain_level);
  amp_envelope2.release(amp_envelope_release);
  amp_envelope3.attack(amp_envelope_attack);
  amp_envelope3.decay(amp_envelope_decay);
  amp_envelope3.sustain(amp_sustain_level);
  amp_envelope3.release(amp_envelope_release);
  amp_envelope4.attack(amp_envelope_attack);
  amp_envelope4.decay(amp_envelope_decay);
  amp_envelope4.sustain(amp_sustain_level);
  amp_envelope4.release(amp_envelope_release);
  amp_envelope5.attack(amp_envelope_attack);
  amp_envelope5.decay(amp_envelope_decay);
  amp_envelope5.sustain(amp_sustain_level);
  amp_envelope5.release(amp_envelope_release);
  amp_envelope6.attack(amp_envelope_attack);
  amp_envelope6.decay(amp_envelope_decay);
  amp_envelope6.sustain(amp_sustain_level);
  amp_envelope6.release(amp_envelope_release);
  amp_envelope7.attack(amp_envelope_attack);
  amp_envelope7.decay(amp_envelope_decay);
  amp_envelope7.sustain(amp_sustain_level);
  amp_envelope7.release(amp_envelope_release);
  amp_envelope8.attack(amp_envelope_attack);
  amp_envelope8.decay(amp_envelope_decay);
  amp_envelope8.sustain(amp_sustain_level);
  amp_envelope8.release(amp_envelope_release);
  filter_cutoff = (cutoff_pot * cutoff_inc) + 50; // calculate filter cutoff (50 - 15000 Hz)
  filter_resonance = (resonance_pot * resonance_inc) + 0.7; // calculate filter resonance gain (0.7 - 5)
  filter1.frequency(filter_cutoff); // update parameters for all filters
  filter1.resonance(filter_resonance);
  filter2.frequency(filter_cutoff);
  filter2.resonance(filter_resonance);
  filter3.frequency(filter_cutoff);
  filter3.resonance(filter_resonance);
  filter4.frequency(filter_cutoff);
  filter4.resonance(filter_resonance);
  filter5.frequency(filter_cutoff);
  filter5.resonance(filter_resonance);
  filter6.frequency(filter_cutoff);
  filter6.resonance(filter_resonance);
  filter7.frequency(filter_cutoff);
  filter7.resonance(filter_resonance);
  filter8.frequency(filter_cutoff);
  filter8.resonance(filter_resonance);
  osc2_detune = (detune_pot * detune_inc) + 1; // calculate oscillator 2 detune (0 - 0.0295%, or half a cent)
  osc1B.frequency(voice_freqs[0] * osc2_detune); // update oscillator 2 frequencies
  osc2B.frequency(voice_freqs[1] * osc2_detune);
  osc3B.frequency(voice_freqs[2] * osc2_detune);
  osc4B.frequency(voice_freqs[3] * osc2_detune);
  osc5B.frequency(voice_freqs[4] * osc2_detune);
  osc6B.frequency(voice_freqs[5] * osc2_detune);
  osc7B.frequency(voice_freqs[6] * osc2_detune);
  osc8B.frequency(voice_freqs[7] * osc2_detune);

  // Polyphonic voice allocation logic (warning may cause headache, refer to diagram of voice allocation algorithm for a high level overview)
  for(int i = 0; i < voices; i++){ // look at each voice in the release queue
    if((release_queue[i] > 0) && (release_queue_count > 0)){ // voices are stored as 1 - n in release queue, so 0 means no voice is there
      if(!check_amp_envelope(release_queue[i] - 1)){ // if this voice is no longer releasing
        memmove(&release_queue[i], &release_queue[i + 1], (voices - 1 - i) * sizeof(int)); // delete it from the queue (even if it's not next up in queue since user may have changed the release pot)
        release_queue_count--;
        release_queue[release_queue_count] = 0; // set new open spot in queue to 0
      }
    }
  }
  if(MIDI.read()){ // if there is an incoming MIDI message
    midi_type = MIDI.getType();
    if((midi_type == midi::NoteOn) || (midi_type == midi::NoteOff)){ // if it is a note off or note on message (only messages we care about)
      midi_note = MIDI.getData1(); // extract which note has been pressed/released
      if(midi_type == midi::NoteOn){ // if the key has been pressed
        for(int i = 0; i < 128; i++){ // look at each note in the MIDI stack
          if(midi_stack[i] == midi_note){ // if the note pressed is currently still in the stack from a previous press
            memmove(&midi_stack[i], &midi_stack[i + 1], (127 - i) * sizeof(int)); // delete it from the stack since it now has highest priority
            midi_stack_index--;
            midi_stack[midi_stack_index] = 0; // initialize new empty spot in stack to 0
            break;
          }
        }

        frequency = freq_table[midi_note]; // find the respective frequency for the note that has been pressed

        for(int i = 0; i < voices; i++){ // look at the status of every voice
          if((voices_on[i] == false) && !check_amp_envelope(i)){ // if there are any voices that aren't being used (note is no longer pressed and its envelope is not releasing)
            voices_on[i] = true; // use this voice for the newly pressed note
            voice_notes[i] = midi_note;
            voice_freqs[i] = frequency;
            notes_on_count++;
            update_note(i); // update the note this voice is playing
            update_amp_envelope(i, true); // start the envelope's attack stage
            break; // exit loop since a voice was found for the newly pressed note
          }
          if(i == (voices - 1)){ // if every voice has been checked and all either currently still have their notes pressed or their envelopes are still releasing
            if(release_queue_count > 0){ // if there is at least one voice that is releasing (also meaning its note is not pressed)
              
              new_voice = release_queue[0] - 1; // steal the voice next up in the release queue (the oldest one)
              voices_on[new_voice] = true; // use this voice for the newly pressed note
              voice_notes[new_voice] = midi_note;
              voice_freqs[new_voice] = frequency;
              notes_on_count++;
              memmove(&release_queue[0], &release_queue[1], (voices - 1) * sizeof(int)); // delete the stolen voice from the release queue
              release_queue_count--;
              release_queue[release_queue_count] = 0;
              update_note(new_voice); // update the note this voice is playing
              update_amp_envelope(new_voice, true); // start the envelope's attack stage
            }else{ // last resort, if the notes of all voices are still being held
              found_match = false;
              for(int j = 0; j < 128; j++){ // look through the MIDI stack
                if(found_match == true){
                  break; // exit loop since a voice to steal has been found
                }
                if((notes_pressed[midi_stack[j]] == true)){ // find the oldest note still being held down
                  for(int k = 0; k < voices; k++){ // look through all voices
                    if(voice_notes[k] == midi_stack[j]){ // find the voice playing that note
                      notes_on[midi_stack[j]] = false; // steal this voice to use for the newly pressed note
                      voice_notes[k] = midi_note;
                      voice_freqs[k] = frequency;
                      update_note(k); // update the note this voice is playing
                      update_amp_envelope(k, true); // start the envelope's attack stage
                      found_match = true; // indicate that a voice to steal has been found
                      break; // exit loop since a voice to steal has been found
                    }
                  }
                }
              }
            }
          }
        }
        notes_pressed[midi_note] = true; // indicate the new note is currently being pressed
        notes_on[midi_note] = true; // indicate the new note is being held down and currently being played by a voice
        notes_pressed_count++;
        if(midi_stack_index > 127){ // if the MIDI stack is full (unlikely unless you can hold 128 keys down at once)
          midi_stack_index = 0;
          memset(midi_stack_ptr, 0, 128 * sizeof(int)); // clear the stack
        }

        midi_stack[midi_stack_index] = midi_note; // add the newly pressed note to the MIDI stack
        midi_stack_index++;
      }else{ // if the key is instead being released
        notes_pressed[midi_note] = false; // indicate this note is no longer being pressed
        notes_pressed_count--;

        if((notes_pressed_count >= voices) && (notes_on[midi_note] == true)) { // if there are currently other notes still held down
            notes_on[midi_note] = false; // indicate this note is no longer being pressed and is not being played by a voice
          
            found_match = false;
            for(int i = midi_stack_index; i >= 0; i--){ // find the next note in the midi stack still held down that is currently not being played by a voice
                if(found_match == true){
                  break; // if a note to revive has been found, exit the loop
                }
                
                if((notes_pressed[midi_stack[i]] == true) && (notes_on[midi_stack[i]] == false)){ // if a note in the stack is currently being pressed but not being played by a voice
                    for(int j = 0; j < voices; j++){ // look through all voices
                      if((voice_notes[j] == midi_note) && voices_on[j]){ // find the voice playing that note in the MIDI stack
                        voice_notes[j] = midi_stack[i]; // use that voice to play the top note in the stack
                        frequency = freq_table[midi_stack[i]]; 
                        notes_on[midi_stack[i]] = true; // indicate that note is now currently being played
                        voice_freqs[j] = frequency;
                        update_note(j); // update the note this voice is playing
                        update_amp_envelope(j, true); // start the envelope's attack phase
                        found_match = true; // indicate the next note in the MIDI stack has been revived
                        break;
                      }
                    }
                }
            }
        }else{ // if no notes are being held down that are currently not being played by a voice
          notes_on[midi_note] = false;
          for(int i = 0; i < voices; i++){ // look at every voice in release queue
            if((voice_notes[i] == midi_note) && voices_on[i]){ // find the voice associated with the key being released
              release_queue[release_queue_count] = i + 1; // add that voice to the release queue
              release_queue_count++;
              update_amp_envelope(i, false); // start the envelope's release phase
              voices_on[i] = false; // indicate this voice is no longer playing a note being held down
              notes_on_count--;
              break;
            }
          }
          if(notes_pressed_count == 0){ // if no keys are being held down, clear the MIDI stack
            midi_stack_index = 0;
            memset(midi_stack_ptr, 0, 128 * sizeof(int));
          }
        }
      }
    }
  }
  waveform_switch_stateA = digitalRead(waveform_switchA1); // check to see if waveform change encoder has been rotated for either oscillator
  waveform_switch_stateB = digitalRead(waveform_switchB1);

  if(waveform_switch_stateA != prev_waveform_switch_stateA){ // if the encoder was just rotated, change the waveform
    if(waveform_switch_stateA == HIGH){
      if(waveform_switch_stateA != digitalRead(waveform_switchA2)){
        if(osc_waveforms[0] == 0){
          osc_waveforms[0] = waveforms - 1;
        }else{
          osc_waveforms[0]--;
        }
      }else{
        if(osc_waveforms[0] == (waveforms - 1)){
          osc_waveforms[0] = 0;
        }else{
          osc_waveforms[0]++;
        }
      }
      osc1A.begin(waveform_lookup[osc_waveforms[0]]); // update all oscillators to use the new waveform
      osc2A.begin(waveform_lookup[osc_waveforms[0]]);
      osc3A.begin(waveform_lookup[osc_waveforms[0]]);
      osc4A.begin(waveform_lookup[osc_waveforms[0]]);
      osc5A.begin(waveform_lookup[osc_waveforms[0]]);
      osc6A.begin(waveform_lookup[osc_waveforms[0]]);
      osc7A.begin(waveform_lookup[osc_waveforms[0]]);
      osc8A.begin(waveform_lookup[osc_waveforms[0]]);
    }
  }

  if(waveform_switch_stateB != prev_waveform_switch_stateB){ // repeat for oscillator B
    if(waveform_switch_stateB == HIGH){
      if(waveform_switch_stateB != digitalRead(waveform_switchB2)){
        if(osc_waveforms[1] == 0){
          osc_waveforms[1] = waveforms - 1;
        }else{
          osc_waveforms[1]--;
        }
      }else{
        if(osc_waveforms[1] == (waveforms - 1)){
          osc_waveforms[1] = 0;
        }else{
          osc_waveforms[1]++;
        }
      }
      osc1B.begin(waveform_lookup[osc_waveforms[1]]);
      osc2B.begin(waveform_lookup[osc_waveforms[1]]);
      osc3B.begin(waveform_lookup[osc_waveforms[1]]);
      osc4B.begin(waveform_lookup[osc_waveforms[1]]);
      osc5B.begin(waveform_lookup[osc_waveforms[1]]);
      osc6B.begin(waveform_lookup[osc_waveforms[1]]);
      osc7B.begin(waveform_lookup[osc_waveforms[1]]);
      osc8B.begin(waveform_lookup[osc_waveforms[1]]);
    }
  }
  prev_waveform_switch_stateA = waveform_switch_stateA;
  prev_waveform_switch_stateB = waveform_switch_stateB;
}