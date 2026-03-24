#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ─────────────────────────────────────────────
   CONFIGURATION
   ───────────────────────────────────────────── */

// Numeric address of heap start in SRAM (integer for safe arithmetic)
// Chosen to sit safely above .data and .bss sections
#define HEAP_START_ADDR  0x20001000UL
// Total heap size: 4 KB
#define HEAP_SIZE        (4 * 1024)
// Each block is 16 bytes
#define BLOCK_SIZE       16
// Total number of blocks = 4096 / 16 = 256
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

/* ─────────────────────────────────────────────
   BLOCK MAP
   block_map[i] = 0 → block i is FREE
   block_map[i] = 1 → block i is IN USE
   ───────────────────────────────────────────── */
static uint8_t block_map[BLOCK_COUNT];


/* ─────────────────────────────────────────────
   heap_init()
   Must be called once in main() before any
   allocation. Marks all blocks as free.
   ───────────────────────────────────────────── */
void heap_init(void) {
    memset(block_map, 0, sizeof(block_map));
}


/* ─────────────────────────────────────────────
   heap_alloc(size_t size)
   Finds enough contiguous free blocks, marks
   them used, and returns the SRAM address
   as a void pointer. Returns NULL if there
   is not enough contiguous space.
   ───────────────────────────────────────────── */
void* heap_alloc(size_t size) {

    // Reject zero-size requests
    if (size == 0) return NULL;

    // Round up to find how many 16-byte blocks are needed
    // e.g. size=20 → (20+15)/16 = 2 blocks
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    // Scan block_map for 'blocks_needed' consecutive free blocks
    for (size_t i = 0; i <= BLOCK_COUNT - blocks_needed; i++) {

        // Check if blocks i, i+1, ..., i+(blocks_needed-1) are all free
        size_t found = 1;
        for (size_t j = 0; j < blocks_needed; j++) {
            if (block_map[i + j] != 0) {
                found = 0;
                break; // This run is broken, try next starting point
            }
        }

        // If a contiguous free run was found, mark all those blocks as used
        if (found) {
            for (size_t j = 0; j < blocks_needed; j++) {
                block_map[i + j] = 1;
            }

            // Integer arithmetic first, then cast to pointer at the last moment
            return (void*)(HEAP_START_ADDR + i * BLOCK_SIZE);
        }
    }

    // No contiguous run found — heap is full or too fragmented
    return NULL;
}


/* ─────────────────────────────────────────────
   heap_free(void* ptr)
   Validates the pointer, finds its block index
   via integer arithmetic, then marks all
   consecutive used blocks as free.
   ───────────────────────────────────────────── */
void heap_free(void* ptr) {

    // Ignore NULL (safe behavior, mirrors standard free)
    if (ptr == NULL) return;

    // Cast pointer to integer for arithmetic — this is what uintptr_t is for
    uintptr_t addr = (uintptr_t)ptr;

    // Validate: pointer must lie inside our heap region
    if (addr < HEAP_START_ADDR || addr >= HEAP_START_ADDR + HEAP_SIZE) {
        return; // Out of range, do nothing
    }

    // Validate alignment: must land exactly on a block boundary
    // Both operands are integers so subtraction and modulo are valid
    if ((addr - HEAP_START_ADDR) % BLOCK_SIZE != 0) {
        return; // Misaligned pointer, do nothing
    }

    // Calculate which block index this address corresponds to
    // Again, pure integer arithmetic throughout
    size_t block_index = (addr - HEAP_START_ADDR) / BLOCK_SIZE;

    // Free all consecutive used blocks starting from block_index
    // We stop at the first block already marked free — that signals
    // the end of this allocation since heap_alloc always marks
    // a contiguous run of 1s
    while (block_index < BLOCK_COUNT && block_map[block_index] == 1) {
        block_map[block_index] = 0;
        block_index++;
    }
}