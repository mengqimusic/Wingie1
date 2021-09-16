declare name        "Wingie";
declare version     "1.8";
declare author      "Meng Qi";
declare license     "GPL";
declare copyright   "(c)Meng Qi 2020";
declare date		"2020-09-30";
declare editDate    "2021-03-13";

//-----------------------------------------------
// Wingie
//-----------------------------------------------

import("stdfaust.lib"); 

nHarmonics = 9;
decay = hslider("decay", 5, 0.1, 10, 0.01) : si.smoo;
input_gain = hslider("input_gain", 0.25, 0, 3, 0.01) : ba.lin2LogGain : si.smoo;
input_gain_factor = hslider("input_gain_factor", 1,0,2,0.01) : ba.lin2LogGain;
output_gain = 1 : ba.lin2LogGain;
left_threshold = hslider("left_threshold", 0.1, 0, 1, 0.01);
right_threshold = hslider("right_threshold", 0.1, 0, 1, 0.01);
amp_follower_decay = 0.025;
resonator_input_gain = hslider("resonator_input_gain", 0.1, 0, 1, 0.01) : ba.lin2LogGain;
resonator_output_gain = hslider("resonator_output_gain", 0.4, 0, 1, 0.01) : ba.lin2LogGain;
//hp_cutoff = hslider("hp_cutoff", 85, 35, 500, 0.1);
bar_factor = 0.44444;

mix = hslider("mix", 1, 0, 1, 0.01) : si.smoo;

vol_wet = mix;
vol_dry = (1 - mix);

note0 = hslider("note0", 36, 12, 96, 1);
note1 = hslider("note1", 36, 12, 96, 1);
route0 = hslider("route0", 0, 0, 4, 1);
route1 = hslider("route1", 0, 0, 4, 1);

env_mode_change = 1 - en.ar(0.002, 0.5, button("mode_changed"));
env_mute(t) = 1 - en.asr(0.25, 1., 0.25, t);

bar_ratios(freq, n) = freq * bar_factor * pow((n + 1) + 0.5, 2);
int_ratios(freq, n) = freq * (n + 1);
//odd_ratios(freq, n) = freq * (2 * n + 1);
//cymbal_808(n) = 130.812792, 193.957204, 235.501256, 333.053319, 344.076511, 392.438376, 509.742979, 581.871611, 706.503769, 999.16, 1032.222378, 1529.218338: ba.selectn(12, n); // chromatic
serge(n) = 62, 115, 218, 411, 777, 1500, 2800, 5200, 11000 : ba.selectn(nHarmonics, n);

poly(n) = a, a * 2, a * 3, b, b * 2, b * 3, c, c * 2, c * 3 : ba.selectn(nHarmonics, n)
with
{
    a = vslider("poly_note_0", 36, 24, 96, 1) : ba.midikey2hz;
    b = vslider("poly_note_1", 36, 24, 96, 1) : ba.midikey2hz;
    c = vslider("poly_note_2", 36, 24, 96, 1) : ba.midikey2hz;
};

//bianzhong(n) = 212.3, 424.6, 530.8, 636.9, 1061.6, 1167.7, 2017.0, 2335.5, 2653.9, 3693 : ba.selectn(10, n);
//cymbal_808(n) = 205.3, 304.4, 369.6, 522.7, 540, 615.9, 800, 913.2, 1108.8, 1568.1 : ba.selectn(10, n); // original
//circular_membrane_ratios(n) = 1, 1.59, 2.14, 2.30, 2.65, 2.92, 3.16, 3.50, 3.60, 3.65 : ba.selectn(10, n);

note_ratio(note) = pow(2., note / 12);

f(note, n, s) = bar_ratios(ba.midikey2hz(note), n),
                int_ratios(ba.midikey2hz(note), n),
                //odd_ratios(ba.midikey2hz(note), n),
                //cymbal_808(n) * note_ratio(note - 48),
                serge(n),
                poly(n)
                : ba.selectn(4, s);

r(note, index, source) = pm.modeFilter(a, b, ba.lin2LogGain(c))
  with
{
  a = min(f(note, index, source), 16000);
  b = env_mode_change * decay + 0.05;
  c = env_mute(button("mute_%index"));
};

process = _,_
    : fi.dcblocker, fi.dcblocker
    : (_ * input_gain_factor), (_ * input_gain_factor)
    : (_ <: attach(_, _ : an.amp_follower(amp_follower_decay) : _ > left_threshold : hbargraph("left_trig", 0, 1))),
      (_ <: attach(_, _ : an.amp_follower(amp_follower_decay) : _ > right_threshold : hbargraph("right_trig", 0, 1)))
        : (_ * input_gain * env_mode_change), (_ * input_gain * env_mode_change)
            <: (_ * resonator_input_gain : _ * vol_wet : fi.lowpass(1, 4000) <: hgroup("left", sum(i, nHarmonics, r(note0, i, route0))) : _ * resonator_output_gain),
               (_ * resonator_input_gain : _ * vol_wet : fi.lowpass(1, 4000) <: hgroup("right", sum(i, nHarmonics, r(note1, i, route1))) : _ * resonator_output_gain),
               (_ * vol_dry),
               (_ * vol_dry)
                //:> co.limiter_1176_R4_mono, co.limiter_1176_R4_mono
                :> ef.cubicnl(0.01, 0), ef.cubicnl(0.01, 0)
                    : (_ * output_gain), (_ * output_gain)
                        ;