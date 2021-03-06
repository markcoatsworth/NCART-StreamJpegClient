#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//Liblo includes
#include "lo/lo.h"
#include "osc_handlers.h"
#include "globals.h"

//Any server errors?
void error(int num, const char *msg, const char *path) {
    printf("liblo server error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}

/* catch any incoming messages and display them. returning 1 means that the
 * message has not been fully handled and the server should try other methods */
int generic_handler(const char *path, const char *types, lo_arg **argv,
                    int argc, void *data, void *user_data) {
    int i;

    printf("path: <%s>\n", path);
    for (i=0; i<argc; i++) {
        printf("arg %d '%c' ", i, types[i]);
        lo_arg_pp((lo_type) types[i], argv[i]);
        printf("\n");
    }
    printf("\n");
    fflush(stdout);

    return 1;
}


int foo_handler(const char *path, const char *types, lo_arg **argv, int argc,
                void *data, void *user_data) {
    /* example showing pulling the argument values out of the argv array */
    printf("%s <- f:%f, i:%d\n\n", path, argv[0]->f, argv[1]->i);
    fflush(stdout);

    return 0;
}

int quit_handler(const char *path, const char *types, lo_arg **argv, int argc,
                 void *data, void *user_data) {
    done = 1;
    printf("quiting\n\n");
    fflush(stdout);

    return 0;
}

