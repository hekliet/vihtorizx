#include <SDL2/SDL.h>
#include <string.h>

#include "globals.h"

#define NUM_KEYS 40
#define SILENCE_NUMBYTES (((102 * HOST_SAMPLE_RATE) >> 8) & 0xFFFFFFF8)

SDL_AudioDeviceID audio_dev;
volatile int presentation_sentinel;
uint8_t presentation_keys[NUM_KEYS];

static SDL_Window *win;
static SDL_Renderer *renderer;

static const unsigned scancodes[NUM_KEYS] = {
    SDL_SCANCODE_LSHIFT, SDL_SCANCODE_Z,      SDL_SCANCODE_X, SDL_SCANCODE_C, SDL_SCANCODE_V,
    SDL_SCANCODE_A,      SDL_SCANCODE_S,      SDL_SCANCODE_D, SDL_SCANCODE_F, SDL_SCANCODE_G,
    SDL_SCANCODE_Q,      SDL_SCANCODE_W,      SDL_SCANCODE_E, SDL_SCANCODE_R, SDL_SCANCODE_T,
    SDL_SCANCODE_1,      SDL_SCANCODE_2,      SDL_SCANCODE_3, SDL_SCANCODE_4, SDL_SCANCODE_5,
    SDL_SCANCODE_0,      SDL_SCANCODE_9,      SDL_SCANCODE_8, SDL_SCANCODE_7, SDL_SCANCODE_6,
    SDL_SCANCODE_P,      SDL_SCANCODE_O,      SDL_SCANCODE_I, SDL_SCANCODE_U, SDL_SCANCODE_Y,
    SDL_SCANCODE_RETURN, SDL_SCANCODE_L,      SDL_SCANCODE_K, SDL_SCANCODE_J, SDL_SCANCODE_H,
    SDL_SCANCODE_SPACE,  SDL_SCANCODE_RSHIFT, SDL_SCANCODE_M, SDL_SCANCODE_N, SDL_SCANCODE_B
};
static Uint32 pixelbuf[PIXELS_COUNT];

void *presentation_start(void *unused) {
    SDL_Init(SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_DisplayMode mode;
    SDL_GetCurrentDisplayMode(0, &mode);
    win = SDL_CreateWindow("Vihtori ZX", (mode.w - SURFACE_WIDTH) / 2, 0, SURFACE_WIDTH, SURFACE_HEIGHT, 0);
    renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC);
    SDL_Texture *texture =
        SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, PIXELS_WIDTH, PIXELS_HEIGHT);
    SDL_AudioSpec want = {HOST_SAMPLE_RATE, AUDIO_F32LSB, 1, 0, 4096, 0, 0, NULL, NULL};
    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, NULL, 0);
    SDL_PauseAudioDevice(audio_dev, 0);

    {
        float silence[SILENCE_NUMBYTES >> 2] = {0};
        while (SDL_GetQueuedAudioSize(audio_dev) < SILENCE_NUMBYTES)
            SDL_QueueAudio(audio_dev, silence, SILENCE_NUMBYTES);
    }

    SDL_Event ev;
    for (;;) {
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT || (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE))
                presentation_sentinel = 1;
        }

        if (presentation_sentinel) break;

        void *dest;
        int pitch;
        if (SDL_LockTexture(texture, NULL, &dest, &pitch)) {
            fprintf(stderr, "%s\n", SDL_GetError());
            presentation_sentinel = 1;
        } else {
            memcpy(dest, pixelbuf, PIXELS_COUNT * 4);
            SDL_UnlockTexture(texture);
        }
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }
    SDL_CloseAudioDevice(audio_dev);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return NULL;
}

void presentation_refresh_keys(void) {
    const uint8_t *kbstate = SDL_GetKeyboardState(NULL);
    for (int i = 0; i < NUM_KEYS; i++) presentation_keys[i] = kbstate[scancodes[i]];
}

void presentation_blit(Uint32 *pixels) { memcpy(pixelbuf, pixels, PIXELS_COUNT * 4); }

uint64_t presentation_perf_freq(void) { return SDL_GetPerformanceFrequency(); }

uint64_t presentation_perf_counter(void) { return SDL_GetPerformanceCounter(); }
