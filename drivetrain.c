#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "drivetrain.h"

const double RATIO_MARGIN_OF_ERROR = 0.000001;

//=BST Helper Functions Declaration==============================================================================

/* Nodes for Binary Search tree
 * stores pointer to elm in gear array
 */
typedef struct Node{
    struct Node* leftC;
    struct Node* rightC;
    uint16_t* data;
}Node_t;

// input param struct for tree traversal
typedef struct TreeWalker_in{
    Node_t* root;
    double* target_ratio;
    uint16_t* in_cog;
    bool cogISfront;
}TreeWalker_in_t;

// output param struct for tree traversal
typedef struct TreeWalker_out{
    double out_ratio;
    uint16_t* out_cog;
    bool success_flag;
}TreeWalker_out_t;

/* recursive function to convert a sorted descending array
 * into a BST on the heap.
 * fist call must have start as 0;
 * and end as array length - 1 */
Node_t* sarray2bst(uint16_t* array, int start, int end);

/* deletes a BST created by sarray2bst() off the heap */
void bst_delete(Node_t* root);

/* recursive part of find_best_ratio()s
 * walks the BST
 * fist call must set the in.root to top of BST.
 * fist call must set out.success_flag = false*/
void find_best_ratio_helper(TreeWalker_in_t* in, TreeWalker_out_t* out);

/* finds a gear in the BST to create the best ratio for a fixed gear.
 * returns 1 if a valid ratio if found; else returns 0.
 * haveCurBest => start up var; set false if no valid ratio has been found yet
 * bst => the BST to search
 * targetRatio => the target Ratio
 * in_cog => fixed gear to combine with BST gear in creating ratio; [front | rear] set by cogISfront
 * cogISfront => true for {in_cog is front; BST is rear}
 *                false for {in_cog is rear; BST is front
 * curBest => gets updated if a valid ratio is found, and it is better then current best.
 *             or set if valid ratio is found and 'haveCurBest' == false
 */
unsigned int find_best_ratio(bool haveCurBest, Node_t* bst, double* const targetRatio,
        uint16_t* const in_cog, const bool cogISfront, DrivetrainOut_t* curBest);

//=Drivetrain============================================================================

bool drivetrain_calc(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        DrivetrainOut_t* out){
    unsigned int haveCurBest = 0; //add 1 by inner loop on valid ratio is found
    Node_t* bst = NULL;

    // BST the larger array
    bool cogISfront = frontLen < rearLen; //will BST larger, and loop on smaller
    if(cogISfront){
        bst = sarray2bst(rearBuff, 0, rearLen - 1); //rear is bigger
    }
    else{
        bst = sarray2bst(frontBuff, 0, frontLen - 1); //front is bigger
    }

    //loop through smaller list
    if(cogISfront){
        //loop Front; BST Rear
        uint8_t i;
        for(i = 0; i < frontLen; i++){
            haveCurBest += find_best_ratio(haveCurBest, bst, targetRatio, &frontBuff[i], cogISfront, out);
        }
    }
    else{
        //loop Rear; BST Front
        uint8_t i;
        for(i = 0; i < rearLen; i++){
            haveCurBest += find_best_ratio(haveCurBest, bst, targetRatio, &rearBuff[i], cogISfront, out);
        }
    }

    //clean up
    bst_delete(bst);

    return haveCurBest > 0;
}

//helper print function used by drivetrain_shift()
void printShift(unsigned int count, uint16_t* f, uint16_t* r){
    double ratio = (double)*f/(double)*r;
    printf("%d - f:%d r:%d ratio: %.3f\n",count,*f,*r,ratio);
}

double drivetrain_shift(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        uint16_t* frontPos, uint16_t* rearPos){
    DrivetrainOut_t out;
    unsigned int count = 1;

    //find optimal gear configuration
    bool pass = drivetrain_calc(targetRatio, frontBuff, frontLen, rearBuff, rearLen, &out);

    //check that ratio was found
    if(!pass){
        printf("Target Ratio Could not be found\nEND\n");
        return 0; //exit early
    }

    printf("f:%d r: %d ratio: %.3f\n",*out.front, *out.rear, out.ratio);

    //shift front gear
    while(frontPos != out.front){
        if(frontPos < out.front){
            printShift(count++, frontPos++, rearPos);
        }
        else{
            printShift(count++, frontPos--, rearPos);
        }
    }
    //shift rear gear
    while(rearPos != out.rear){
        if(rearPos < out.rear){
            printShift(count++, frontPos, rearPos++);
        }
        else{
            printShift(count++, frontPos, rearPos--);
        }
    }
    //final print
    printShift(count, frontPos, rearPos);

    return out.ratio;
}

//=Helper Definitions=====================================================================

Node_t* sarray2bst(uint16_t* array, int start, int end){
    //base case
    if(start > end){return NULL;}

    //make mid root
    uint8_t mid = (start + end) / 2;
    Node_t* root = malloc(sizeof(Node_t));
    root->data = &(array[mid]);
    root->rightC = NULL;
    root->leftC = NULL;

    #ifdef DEBUG
    printf("NODE [%d]\n",array[mid]);
    #endif

    //make right
    #ifdef DEBUG
    printf("left\n");
    #endif
    root->leftC = sarray2bst(array, mid + 1, end);
    //make left
    #ifdef DEBUG
    printf("Right\n");
    #endif
    root->rightC = sarray2bst(array, start, mid - 1);

    #ifdef DEBUG
    printf("END\n");
    #endif
    return root;
}

void bst_delete(Node_t* root){
    if(root->leftC != NULL){bst_delete(root->leftC);}
    if(root->rightC != NULL){bst_delete(root->rightC);}
    free(root);
}

void find_best_ratio_helper(TreeWalker_in_t* in, TreeWalker_out_t* out){
    //exit case on NULL
    if(in->root == NULL){return;}

    //do calc
    double test_ratio; 
    if(in->cogISfront){
        test_ratio = (double)(*in->in_cog)/(double)(*in->root->data);
    }
    else{
        test_ratio = (double)(*in->root->data)/(double)(*in->in_cog);
    }

    #ifdef DEBUG
    printf("Fixed: %d, BST: %d, ratio:%f\n",*in->in_cog, *in->root->data, test_ratio);
    #endif

    //exit case on perfect
    if(fabs(test_ratio - *(in->target_ratio)) < RATIO_MARGIN_OF_ERROR)
    {
        //update and return
        out->out_cog = in->root->data;
        out->out_ratio = test_ratio;
        out->success_flag = true;
        return;
    }

    //Polarity depends on if BST is front or rear gear
    //if bst is rear and want bigger number: GO LEFT
    //if bst is front and want smaller number: GO LEFT
    //else go right

    bool bust = test_ratio > *in->target_ratio;
    if(!bust){
        //update output with best so far
        out->success_flag = true;
        out->out_cog = in->root->data;
        out->out_ratio = test_ratio;
        #ifdef DEBUG
        printf("\tSAVE!\n");
        #endif
    }
    #ifdef DEBUG
    else{
        printf("\tBUST!\n");
    }
    #endif

    //LEFT
    //  {BST is Rear  AND want bigger} OR {BST is Front AND want smaller}
    if((in->cogISfront && !bust) || (!in->cogISfront && bust)){
        in->root = in->root->leftC;
        #ifdef DEBUG
        printf("\twent left\n");
        #endif
        find_best_ratio_helper(in, out);
    }
    //RIGHT
    else{
        in->root = in->root->rightC;//set next node as right
        #ifdef DEBUG
        printf("\twent right\n");
        #endif
        find_best_ratio_helper(in, out); //recurs right
    }
}

unsigned int find_best_ratio(bool haveCurBest, Node_t* bst, double* const targetRatio,
        uint16_t* const in_cog, const bool cogISfront, DrivetrainOut_t* curBest){
    //set up tree walker
    TreeWalker_in_t in;
    TreeWalker_out_t out;
    in.root = bst;
    in.target_ratio = targetRatio;
    in.in_cog = in_cog;
    in.cogISfront = cogISfront;
    out.success_flag = false;

    //walk tree
    find_best_ratio_helper(&in, &out);
    //update best
    if(out.success_flag)
    {
        #ifdef DEBUG
        printf("Best: %f\n",out.out_ratio);
        #endif
        //dont need fabs() because of bust rule
        bool test = (*targetRatio - curBest->ratio) > (*targetRatio - out.out_ratio);
        if(test || !haveCurBest){
            #ifdef DEBUG
            printf("Overwrite as new best\n");
            #endif
            curBest->ratio = out.out_ratio;
            if(cogISfront){
                curBest->front = in_cog;
                curBest->rear = out.out_cog;
            }
            else{
                curBest->front = out.out_cog;
                curBest->rear = in_cog;
            }
        }
    }
    #ifdef DEBUG
    else{printf("ALL BUST\n");}
    #endif

    if(out.success_flag){return 1;}
    else{return 0;}
}

