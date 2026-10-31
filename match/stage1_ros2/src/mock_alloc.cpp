#include <cstdlib>
#include <new>

static unsigned long g_alloc_count = 0;

unsigned long alloc_count() { return g_alloc_count; }
void alloc_reset() { g_alloc_count = 0; }

void* operator new(std::size_t size) {
    g_alloc_count++;
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }