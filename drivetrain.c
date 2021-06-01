#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "drivetrain.h"

//=BST=============================================================================

typedef struct Node{
    struct Node* leftC;
    struct Node* rightC;
    uint16_t data;
}Node_t;

typedef struct TreeWalker{
    Node_t* root;
    double target_ratio;
    double out_ratio;
    uint16_t out_cog;
    uint16_t in_cog;
    bool cogISfront;
    bool success_flag;
}TreeWalker_t;

Node_t* sarray2bst(uint16_t* array, uint8_t start, u_int8_t end){
    //base case
    if(start == end){return NULL;}

    //make mid root
    uint8_t mid =( start + end) / 2;
    Node_t* root = malloc(sizeof(Node_t));
    root->data = array[mid];

    //make right
    root->rightC = sarray2bst(array, mid + 1, end);
    //make left
    root->leftC = sarray2bst(array, start, mid - 1);

    return root;
}

void bst_delete(Node_t* root){
    if(root->leftC != NULL){bst_delete(root->leftC);}
    if(root->rightC != NULL){bst_delete(root->rightC);}
}


void find_best_ratio_helper(TreeWalker_t* meta){
    //exit case on NULL
    if(meta->root == NULL){return;}

    //do calc
    double test_ratio; 
    if(meta->cogISfront){
        test_ratio = (double)(meta->in_cog)/(double)(meta->root->data);
    }
    else{
        test_ratio = (double)(meta->root->data)/(double)(meta->in_cog);
    }

    //exit case on perfect
    if(fabs(test_ratio - meta->target_ratio) < RATIO_MARGIN_OF_ERROR)
    {
        //update and return
        meta->out_cog = meta->root->data;
        meta->out_ratio = test_ratio;
        return;
    }

    //go right?
    else if(test_ratio < meta->target_ratio){
        //update output with best so far
        meta->success_flag = true;
        meta->out_cog = meta->root->data;
        meta->out_ratio = test_ratio;
        meta->root = meta->root->rightC;//set next node as right
        find_best_ratio_helper(meta); //recurs right
    }

    //BUST!; don't update and go left
    else{
        meta->root = meta->root->leftC;
        find_best_ratio_helper(meta);
    }
}

bool find_best_ratio(Node_t* bst, double* const targetRatio, const uint16_t in_cog, const bool cogISfront, DrivetrainOut_t* curBest){
    //set up tree walker
    TreeWalker_t meta;
    meta.root = bst;
    //FIXME: pase by ref
    meta.target_ratio = *targetRatio;
    meta.in_cog = in_cog;
    meta.success_flag = false;

    //walk tree
    find_best_ratio_helper(&meta);
    //update best
    if(meta.success_flag)
    {
        //dont need fabs() because of bust rule
        if((*targetRatio - curBest->ratio) < (*targetRatio - meta.out_ratio)){
            curBest->ratio = meta.out_ratio;
            if(cogISfront){
                curBest->front = in_cog;
                curBest->rear = meta.out_cog;
            }
            else{
                curBest->front = meta.out_cog;
                curBest->rear = in_cog;
            }
        }
    }
}

//=Drivetrain============================================================================

//FIXME: pass by ref
DrivetrainOut_t calc_drivetrain(double targetRatio,
       uint16_t* frontBuff, uint8_t frontLen,
       uint16_t* rearBuff, uint8_t rearLen){
    // BST the larger array
    DrivetrainOut_t out = {0,0,0};
    bool cogISfront = frontLen < rearLen; //will BST larger, and loop on smaller
    Node_t* bst = NULL;
    if(cogISfront){
        bst = sarray2bst(rearBuff, 0, rearLen - 1); //rear is bigger
    }
    else{
        bst = sarray2bst(frontBuff, 0, frontLen - 1); //front is bigger
    }

    //FIXME; replace with default failing return code
    //prime the first best ratio
    if(cogISfront){
        out.front = frontBuff[0];
        out.rear  = bst->data;
    }
    else{
        out.front = bst->data;
        out.rear  = rearBuff[0];
    }
    out.ratio = (double)out.front / (double)out.rear;

    //loop through smaller list
    if(cogISfront){
        for(uint8_t i = 0; i < frontLen; i++){
            //FIXME: get return code
            find_best_ratio(bst, &targetRatio, frontBuff[i], cogISfront, &out);
        }
    }
    else{
        for(uint8_t i = 0; i < rearLen; i++){
            //FIXME: get return code
            find_best_ratio(bst, &targetRatio, rearBuff[i], cogISfront, &out);
        }
    }

    //clean up
    bst_delete(bst);

    return out;
}

