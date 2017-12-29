#!/bin/bash

# Usage: ./speed_test.sh [options] program-path [test-paths]
#
# Options:
#   -b <num = 0>
#       Blimit number to pass to program.
#   -t <num = 1>
#       Maximum number of threads.
#   -s <dir>
#       Compares standard output to files in given dierctory. Looks for files
#       with same name as test-paths names and .out extension. Compares only 
#       blimit+1 first lines. Does not output speed data if program output 
#       differs from file.
#   --time
#       Script will calculate program's running time using 'time' command.
#
# Script expects output on error stream from the program consisting of a single
# number signifying running time of the algorithm (except when --time is on).
#
# Output (for every test-path):
#   <test-path> <number-of-threads> <b-limit> <time>

USAGE="For usage read comments in script."
THREADS=1
BLIMIT=0
USETIME=false
SOLS=""
EXEC=""

reInt='^[0-9]+$'
reFlo='^[0-9]+([.][0-9]+)?$'

if [[ $# -eq 0 ]]
then
    echo "$USAGE" >&2
    exit 1
fi

while [[ $# -gt 0 && -z $EXEC ]]; do
case "$1" in
    -t)
        shift
        if (($#))
        then
            THREADS="$1"
            if ! [[ $THREADS =~ $reInt ]]
            then
                echo "Error: threads parameter not a valid number" >&2
                exit 1
            fi
        else
            echo "Error: no threads number specified" >&2
            exit 1
        fi
        shift
        ;;
    -b)
        shift
        if (($#))
        then
            BLIMIT="$1"
            if ! [[ $BLIMIT =~ $reInt ]]
            then
                echo "Error: blimit parameter not a valid number" >&2
                exit 1
            fi
        else
            echo "Error: no blimit number specified" >&2
            exit 1
        fi
        shift
        ;;
    -s)
        shift
        if (($#))
        then
            SOLS="$1"
            if ! [[ -d $SOLS ]]
            then
                echo "Error: solutions path not a directory" >&2
                exit 1
            fi
        else
            echo "Error: no solutions directory specified" >&2
            exit 1
        fi
        shift
        ;;
    --time)
        USETIME=true
        shift
        ;;
    *)
        EXEC="$1"
        if ! [[ -x $EXEC ]]
        then
            echo "Error: program is not a valid executable" >&2
            exit 1
        fi
        shift
        ;;
esac
done

if [[ -z $EXEC ]]
then
    echo "Error: no program specified" >&2
    exit 1
fi

while [[ $# -gt 0 ]]
do
    TEST="$1"
    shift

# check if TEST is a valid file

    TIME=0

    if [[ $USETIME = true ]]
    then
        { time -p "./$EXEC" $THREADS "$TEST" $BLIMIT > temp1.out 2> temp2.out ;
            } 2> temp-time.out
        TIME=`grep "real" temp-time.out | sed 's/[^ ]* //'`
    else
        if ! "./$EXEC" $THREADS "$TEST" $BLIMIT > temp1.out 2> temp2.out
        then
            echo "Error: program did not exit with code 0" >&2
            exit 1
        fi
        TIME=`cat temp2.out`
    fi

    if ! [[ $TIME =~ $reFlo ]]
    then
        echo "Error: program error stream output is not a valid floating point number" >&2
        exit 1
    fi
    
    if ! [[ -z $SOLS ]]
    then 

    TESTFILE=${TEST##*/}
    TESTFILE=${TESTFILE%.*}
    SOLSFILE="${TESTFILE}.out"
    SOLSPATH="${SOLS}${SOLSFILE}"

    if ! [[ -f $SOLSPATH ]]
    then
        echo "Warning: solution file '$SOLSPATH' not found" >&2
    elif ! diff <(head -n $(($BLIMIT + 1)) temp1.out) <(head -n $(($BLIMIT + 1)) "$SOLSPATH")
    then
        echo "Warning: outputs differ for test $FILE" >&2
    else
        echo -e "${TEST}\t${THREADS}\t${BLIMIT}\t${TIME}"
    fi

    else
        echo -e "${TEST}\t${THREADS}\t${BLIMIT}\t${TIME}"
    fi
done

rm -f temp1.out temp2.out temp-time.out
