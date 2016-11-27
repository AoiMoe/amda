#include <memory>
#include <amda.h>
#include <cstdio>

class A {
    static int s_count;
    int m_count;
    bool m_moved = false;
#ifdef SIDE_EFFECT
    void put_(const char *label, const A *from = NULL)
    {
        printf("   %s(%d)", label, m_count);
        if (m_moved)
            printf("(moved)");
        if (from)
            printf(" from %d", from->m_count);
        printf("\n");
    }
    static int get_count_() { return s_count++; }
#else
    void put_(const char *, const A * = NULL) { }
    static int get_count_() { return 0; }
#endif
public:
    ~A() { put_("dtor"); }
    A() : m_count{get_count_()} { put_("default ctor"); }
    A(A &&a) : m_count{get_count_()}
    {
        put_("move ctor", &a);
        a.m_moved = true;
    }
    A(const A &a) : m_count{get_count_()}
    {
        put_("copy ctor", &a);
    }
    A &operator = (A &&a)
    {
        put_("move =", &a);
        a.m_moved = true;
        return *this;
    }
    A &operator = (const A &a)
    {
        put_("copy =", &a);
        return *this;
    }
};

int A::s_count = 0;

int
main()
{
    std::printf("(0)\n");
    auto f1 = AMDA::make_failable<A>();

    std::printf("(1)\n");
    auto f2 = AMDA::make_failable<A>();

    std::printf("(2)\n");
    f1
        .apply([](auto a) {
                std::printf("(2)apply(1)\n");
                {
                    A f3=std::move(a);
                    std::printf("(2)apply(2)\n");
                }
                std::printf("(2)apply(3)\n");
            })
        // Failable<void> with S_NONE state.
        .failure([](auto s) {
                // not reached, because S_NONE is not failure.
                std::printf("(2)failure\n");
            });

    std::printf("(3)\n");
    auto f3 = f2.unwrap();

    std::printf("(4)\n");
    auto f4 = AMDA::make_failable<std::unique_ptr<A>>(std::make_unique<A>());
    f4
        .apply([](auto p) {
                std::printf("(4)apply(1)\n");
                {
                    auto f5 = std::move(p);
                    std::printf("(4)apply(2)\n");
                }
                std::printf("(4)apply(3)\n");
            })
        .failure([](auto s) {
                std::printf("(4)failure\n");
            });

    std::printf("(5)\n");
    AMDA::make_failable<A>()
        // Failable<A> with successful A.
        .apply([](A a) {
                std::printf("(5)apply(1)\n");
                return 42;
            })
        // Failable<int> with successful 42.
        .apply([](int v) {
                std::printf("(5)apply(2): %d\n", v);
                return AMDA::S_BREAK;
            })
        // Failable<void> with S_BREAK state.
        .failure([](auto s) {
                std::printf("(5)failure: %d\n", s);
            });

    std::printf("(6)\n");

    return 0;
}
