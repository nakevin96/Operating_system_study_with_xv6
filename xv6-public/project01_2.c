#include "types.h"
#include "stat.h"
#include "user.h"
#include "fcntl.h"

int
main(int argc, char * argv[])
{
    __asm__("int $128");
    exit();
}