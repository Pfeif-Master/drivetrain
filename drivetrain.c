#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "drivetrain.h"

const double RATIO_MARGIN_OF_ERROR = 0.000001;

//=BST Helper Functions Declaraiton==============================================================================
typedef struct Node{
    struct Node* leftC;
    struct Node* rightC;
    uint16_t* data;
}Node_t;

typedef struct TreeWalker_in{
    Node_t* root;
    double* target_ratio;
    uint16_t* in_cog;
    bool cogISfront;
}TreeWalker_in_t;

typedef struct TreeWalker_out{
    double out_ratio;
    uint16_t* out_cog;
    bool success_flag;
}TreeWalker_out_t;

Node_t* sarray2bst(uint16_t* array, int start, int end);

void bst_delete(Node_t* root);

void find_best_ratio_helper(TreeWalker_in_t* in, TreeWalker_out_t* out);

bool find_best_ratio(bool haveCurBest, Node_t* bst, double* const targetRatio,
        uint16_t* const in_cog, const bool cogISfront, DrivetrainOut_t* curBest);

//=Drivetrain============================================================================

bool drivetrain_calc(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        DrivetrainOut_t* out){
    bool haveCurBest = false; //set true by inner loop when first valid ratio is found
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
        for(uint8_t i = 0; i < frontLen; i++){
            haveCurBest = find_best_ratio(haveCurBest, bst, targetRatio, &frontBuff[i], cogISfront, out);
        }
    }
    else{
        //loop Rear; BST Front
        for(uint8_t i = 0; i < rearLen; i++){
            haveCurBest = find_best_ratio(haveCurBest, bst, targetRatio, &rearBuff[i], cogISfront, out);
        }
    }

    //clean up
    bst_delete(bst);

    return haveCurBest;
}

void printShift(unsigned int count, uint16_t* f, uint16_t* r){
    double ratio = (double)*f/(double)*r;
    printf("%d - f:%d r:%d ratio: %.3f\n",count,*f,*r,ratio);
}

//front and rear Pos pointers get modified
void drivetrain_shift(double* const targetRatio,
        uint16_t* frontBuff, uint8_t frontLen,
        uint16_t* rearBuff, uint8_t rearLen,
        uint16_t* frontPos, uint16_t* rearPos){
    DrivetrainOut_t out;
    unsigned int count = 1;

    //find optimal gear configuration
    bool pass = drivetrain_calc(targetRatio, frontBuff, frontLen, rearBuff, rearLen, &out);

    //check that ratio was found
    if(!pass){
        printf("Target Ratio Could not be found\nEND");
        return; //exit early
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

    //make right
    root->rightC = sarray2bst(array, mid + 1, end);
    //make left
    root->leftC = sarray2bst(array, start, mid - 1);

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

    //exit case on perfect
    if(fabs(test_ratio - *(in->target_ratio)) < RATIO_MARGIN_OF_ERROR)
    {
        //update and return
        out->out_cog = in->root->data;
        out->out_ratio = test_ratio;
        out->success_flag = true;
        return;
    }

    //go right?
    else if(test_ratio < *in->target_ratio){
        //update output with best so far
        out->success_flag = true;
        out->out_cog = in->root->data;
        out->out_ratio = test_ratio;
        in->root = in->root->rightC;//set next node as right
        find_best_ratio_helper(in, out); //recurs right
    }

    //BUST!; don't update and go left
    else{
        in->root = in->root->leftC;
        find_best_ratio_helper(in, out);
    }
}

bool find_best_ratio(bool haveCurBest, Node_t* bst, double* const targetRatio,
        uint16_t* const in_cog, const bool cogISfront, DrivetrainOut_t* curBest){
    //set up tree walker
    TreeWalker_in_t in;
    TreeWalker_out_t out;
    in.root = bst;
    in.target_ratio = targetRatio;
    in.in_cog = in_cog;
    out.success_flag = false;

    //walk tree
    find_best_ratio_helper(&in, &out);
    //update best
    if(out.success_flag)
    {
        //dont need fabs() because of bust rule
        bool test = (*targetRatio - curBest->ratio) > (*targetRatio - out.out_ratio);
        if(test || !haveCurBest){
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

    return out.success_flag;
}

