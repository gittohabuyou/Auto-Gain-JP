#include <obs-module.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define FILTER_ID "auto_gain_jp"
#define LINEAR_FLOOR 0.000001f

struct auto_gain_data {
    float target_db;
    float max_boost_db;
    float max_cut_db;
    float gate_db;
    float attack_ms;
    float release_ms;
    float gain;
    uint32_t sample_rate;
    uint32_t channels;
};

static const char *auto_gain_name(void *unused)
{
    UNUSED_PARAMETER(unused);
    return obs_module_text("AutoGain.Name");
}

static float db_to_linear(float db)
{
    return powf(10.0f, db / 20.0f);
}

static float linear_to_db(float value)
{
    return 20.0f * log10f(fmaxf(value, LINEAR_FLOOR));
}

static void auto_gain_update(void *data, obs_data_t *settings)
{
    struct auto_gain_data *f = data;
    f->target_db = (float)obs_data_get_double(settings, "target_db");
    f->max_boost_db = (float)obs_data_get_double(settings, "max_boost_db");
    f->max_cut_db = (float)obs_data_get_double(settings, "max_cut_db");
    f->gate_db = (float)obs_data_get_double(settings, "gate_db");
    f->attack_ms = (float)obs_data_get_double(settings, "attack_ms");
    f->release_ms = (float)obs_data_get_double(settings, "release_ms");

    f->target_db = fminf(-3.0f, fmaxf(-36.0f, f->target_db));
    f->max_boost_db = fminf(24.0f, fmaxf(0.0f, f->max_boost_db));
    f->max_cut_db = fminf(36.0f, fmaxf(0.0f, f->max_cut_db));
    f->gate_db = fminf(-30.0f, fmaxf(-80.0f, f->gate_db));
    f->attack_ms = fminf(1000.0f, fmaxf(5.0f, f->attack_ms));
    f->release_ms = fminf(3000.0f, fmaxf(20.0f, f->release_ms));
}

static void *auto_gain_create(obs_data_t *settings, obs_source_t *source)
{
    UNUSED_PARAMETER(source);
    struct auto_gain_data *f = bzalloc(sizeof(*f));
    struct obs_audio_info ai = {0};

    f->gain = 1.0f;
    f->sample_rate = obs_get_audio_info(&ai) ? ai.samples_per_sec : 48000;
    f->channels = obs_get_audio_info(&ai) ? get_audio_channels(ai.speakers) : 2;
    if (f->sample_rate == 0) f->sample_rate = 48000;
    if (f->channels == 0 || f->channels > MAX_AV_PLANES) f->channels = 2;

    obs_data_set_default_double(settings, "target_db", -18.0);
    obs_data_set_default_double(settings, "max_boost_db", 12.0);
    obs_data_set_default_double(settings, "max_cut_db", 12.0);
    obs_data_set_default_double(settings, "gate_db", -55.0);
    obs_data_set_default_double(settings, "attack_ms", 80.0);
    obs_data_set_default_double(settings, "release_ms", 500.0);
    auto_gain_update(f, settings);
    return f;
}

static void auto_gain_destroy(void *data)
{
    bfree(data);
}

static struct obs_audio_data *auto_gain_filter_audio(void *data,
                                                       struct obs_audio_data *audio)
{
    struct auto_gain_data *f = data;
    if (!audio || audio->frames == 0)
        return audio;

    double sum_sq = 0.0;
    float peak = 0.0f;
    uint64_t count = 0;

    for (uint32_t ch = 0; ch < f->channels; ch++) {
        if (!audio->data[ch]) continue;
        float *samples = (float *)audio->data[ch];
        for (uint32_t i = 0; i < audio->frames; i++) {
            float x = samples[i];
            sum_sq += (double)x * (double)x;
            peak = fmaxf(peak, fabsf(x));
            count++;
        }
    }
    if (!count) return audio;

    float rms = (float)sqrt(sum_sq / (double)count);
    float desired = 1.0f;

    /* Do not raise the gain when the input is below the noise gate. */
    if (rms >= db_to_linear(f->gate_db))
        desired = db_to_linear(f->target_db) / fmaxf(rms, LINEAR_FLOOR);

    desired = fminf(db_to_linear(f->max_boost_db),
                    fmaxf(db_to_linear(-f->max_cut_db), desired));

    /* Keep the block peak below full scale. */
    if (peak > 0.0f)
        desired = fminf(desired, 0.98f / peak);

    /* Faster gain reduction, slower gain increase. */
    float ms = desired < f->gain ? f->attack_ms : f->release_ms;
    float dt = (float)audio->frames / (float)f->sample_rate;
    float tau = fmaxf(ms / 1000.0f, 0.001f);
    float alpha = 1.0f - expf(-dt / tau);
    f->gain += (desired - f->gain) * alpha;

    if (peak > 0.0f && peak * f->gain > 0.98f)
        f->gain = 0.98f / peak;

    for (uint32_t ch = 0; ch < f->channels; ch++) {
        if (!audio->data[ch]) continue;
        float *samples = (float *)audio->data[ch];
        for (uint32_t i = 0; i < audio->frames; i++)
            samples[i] *= f->gain;
    }
    return audio;
}

static obs_properties_t *auto_gain_properties(void *unused)
{
    UNUSED_PARAMETER(unused);
    obs_properties_t *p = obs_properties_create();
    obs_properties_add_float_slider(p, "target_db", obs_module_text("TargetLevel"), -36.0, -3.0, 0.5);
    obs_properties_add_float_slider(p, "max_boost_db", obs_module_text("MaxBoost"), 0.0, 24.0, 0.5);
    obs_properties_add_float_slider(p, "max_cut_db", obs_module_text("MaxCut"), 0.0, 36.0, 0.5);
    obs_properties_add_float_slider(p, "gate_db", obs_module_text("NoiseGate"), -80.0, -30.0, 1.0);
    obs_properties_add_float_slider(p, "attack_ms", obs_module_text("Attack"), 5.0, 1000.0, 5.0);
    obs_properties_add_float_slider(p, "release_ms", obs_module_text("Release"), 20.0, 3000.0, 10.0);
    return p;
}

struct obs_source_info auto_gain_filter = {
    .id = FILTER_ID,
    .type = OBS_SOURCE_TYPE_FILTER,
    .output_flags = OBS_SOURCE_AUDIO,
    .get_name = auto_gain_name,
    .create = auto_gain_create,
    .destroy = auto_gain_destroy,
    .update = auto_gain_update,
    .get_properties = auto_gain_properties,
    .filter_audio = auto_gain_filter_audio,
};
