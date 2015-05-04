/*
 * Portions Copyright (c) 1987, 1993, 1994
 * The Regents of the University of California.  All rights reserved.
 *
 * Portions Copyright (c) 2003-2010, PostgreSQL Global Development
 * Group
 *
 * Simple conversion to C++ by Andrew Tridgell for ArduPilot. Based on
 * getopt_long.h from ccache
 */
#ifndef GETOPT_LONG_H
#define GETOPT_LONG_H

class GetOptLong {
public:
    struct option {
	const char *name;
	int         has_arg;
	int        *flag;
	int         val;
    };

    GetOptLong(int argc, char *const argv[], const char *optstring, const option * longopts);

    int   opterr;
    int   optind;
    int   optopt;
    int   longindex;
    const char *optarg;

    enum { 
        no_argument=0,
        required_argument=1
    };

    enum error_return {
        BADCH='?',
        BADARG=':'
    };


    int getoption(void);

private:
    int argc;
    char *const *argv;
    const char *optstring;
    const struct option *longopts;
    const char *place;
};

#endif  /* GETOPT_LONG_H */
