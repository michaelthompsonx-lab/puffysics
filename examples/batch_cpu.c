// Same task functions as the native PufferLib CPU/CUDA adapters, no trainer.
#include <stdio.h>
#include <stdlib.h>
#include "pufferlib/puffysics_hover/hover_task.h"

int main(void) {
    enum { COUNT = 64 };
    HoverTask* tasks = (HoverTask*)calloc(COUNT, sizeof(HoverTask));
    if (!tasks) return 1;
    for (int i = 0; i < COUNT; i++)
        hover_init(&tasks[i], hover_default_config(), (uint32_t)i + 1u);
    B3Batch batch = {tasks, sizeof(HoverTask), offsetof(HoverTask, world), COUNT};
    for (int step = 0; step < 120; step++) {
        for (int i = 0; i < COUNT; i++) hover_apply_action(&tasks[i], 1.0f);
        if (!b3_batch_step(batch, 1.0f / 60.0f, 4, NULL)) { free(tasks); return 1; }
    }
    printf("%d independent worlds: first height %.4f, last %.4f\n", COUNT,
        tasks[0].world.bodies[0].position.y, tasks[COUNT-1].world.bodies[0].position.y);
    free(tasks);
    return 0;
}
