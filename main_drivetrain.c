#include <stdio.h>
#include <math.h>
#include "drivetrain.h"

typedef struct testCase{
    char name[50];
    //inputs
    uint16_t* front_cogs;
    uint16_t* rear_cogs;
    uint8_t f_size;
    uint8_t r_size;
    uint16_t* f_start;
    uint16_t* r_start;
    double target_ratio;
    //checks
    double expected_ratio;
} testCase_t;

double calc_ratio(uint16_t f, uint16_t r){
    return (double)f/(double)r;
}

bool run_test(testCase_t* test, int count);

int main(){
    {//Give test case
        int count = 0;
        uint16_t f[2] = {38,30};
        uint16_t r[4] = {28,23,19,16};

        testCase_t given = {"given", f, r, 2, 4, f, r, 1.6, calc_ratio(f[1], r[2])};
        if(!run_test(&given, count++)){return 1;}

        given.r_start = &r[3];
        given.f_start = &f[1];
        if(!run_test(&given, count++)){return 1;}
    }

    {//Edge test case
        int count = 0;
        uint16_t f[2] = {38,30};
        uint16_t r[4] = {28,23,19,16};

        testCase_t perfect = {"edge", f, r, 2, 4, f, r, 1.579, calc_ratio(f[1], r[2])};
        if(!run_test(&perfect, count++)){return 1;}

        testCase_t none = {"edge", f, r, 2, 4, f, r, 1, 0};
        if(!run_test(&none, count++)){return 1;}

        testCase_t far_over = {"edge", f, r, 2, 4, f, r, 5, calc_ratio(f[0], r[3])};
        if(!run_test(&far_over, count++)){return 1;}
    }

    {//BST front test
        int count = 0;
        uint16_t f[10] = {100, 91, 82, 73, 64, 55, 46, 37, 28, 18}; 
        uint16_t r[5]  = {210, 130, 58, 23, 11};

        testCase_t bst = {"BST_F", f, r, 10, 5, &f[7], &r[2], 0, 0};
        for(int i = 0; i < 10; i++){
            bst.target_ratio = calc_ratio(f[5],r[i])+0.0001;
            bst.expected_ratio = calc_ratio(f[5],r[i]);
            if(!run_test(&bst, count++)){return 1;}
        }

    }
}

bool run_test(testCase_t* test, int count){
    printf("\e[35m");
    printf("<<========================================>>\n");
    printf("\t%s_%d\n",test->name, count);
    printf("<<========================================>>\n");

    //Print Params
    printf("\e[36m");
    printf("Front: ");
    for(int i = 0; i < test->f_size; i++){
        if(&test->front_cogs[i] == test->f_start){
            printf("[%d], ",test->front_cogs[i]);
        }
        else{
            printf("%d, ",test->front_cogs[i]);
        }
    }
    printf("\nRear: ");
    for(int i = 0; i < test->r_size; i++){
        if(&test->rear_cogs[i] == test->r_start){
            printf("[%d], ",test->rear_cogs[i]);
        }
        else{
            printf("%d, ",test->rear_cogs[i]);
        }
    }
    printf("\nTarget Ratio: %.3f", test->target_ratio);
    printf("\e[32m");
    printf("\nExpected Ratio: %f\n", test->expected_ratio);
    printf("\e[35m");
    printf("OUTPUT:\n");
    printf("\e[m");

    //Run Test
    double result = drivetrain_shift(&test->target_ratio, test->front_cogs,
            test->f_size, test->rear_cogs, test->r_size,
            test->f_start, test->r_start);
    if(fabs(result - test->expected_ratio) < 0.000001){
        printf("\e[30;42mPASS\e[m\n");
        return true;
    }
    else{
        printf("\e[30;41mFAIL\e[m\n");
        return false;
    }

}
