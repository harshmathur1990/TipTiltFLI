//
// Created by hp on 8/17/2021.
//
#include<iostream>
#define NX 128
#define NY 128

int checkValidCoordinates(int i, int j){
    if ((i < 0 || i >= NX) || (j < 0 || j >= NY)) {
        return 0;
    }
    return 1;
}

int getLinearArrayIndice(int i, int j) {
    return j * NX + i;
}

int* get_param_generic(int ind) {
    int XIND = ind%NX, YIND = ind/NX;
    int *coeff = new int[9];
    int k = 0;
    int i, j, nei_x, nei_y;
    for (j=-1;j<2;j++) {
        for (i=-1;i<2;i++) {
            nei_y = YIND + j;
            nei_x = XIND + i;
            if (nei_x < 0) nei_x += NX;
            else if (nei_x >= NX) nei_x -= NX;
            if (nei_y < 0) nei_y += NY;
            else if (nei_y >= NY) nei_y -= NY;
            coeff[k] = getLinearArrayIndice(nei_x, nei_y);
            k += 1;
        }
    }
    return coeff;
}

int* get_param_hardcoded(int ind) {
    int XIND = ind%NX, YIND = ind/NX;
    int *coeff = new int[9];
    if (YIND == 0){
        if (XIND == 0){
            coeff[0] = 16383;
            coeff[1] = 16256;
            coeff[2] = 16257;
            coeff[3] = 127;
            coeff[4] = 0;
            coeff[5] = 1;
            coeff[6] = 255;
            coeff[7] = 128;
            coeff[8] = 129;
        }
        else if (XIND == 127) {
            coeff[0] = 16382;
            coeff[1] = 16383;
            coeff[2] = 16256;
            coeff[3] = 126;
            coeff[4] = 127;
            coeff[5] = 128;
            coeff[6] = 254;
            coeff[7] = 255;
            coeff[8] = 256;
        }
        else{
            coeff[0] = ind+16255;
            coeff[1] = ind+16256;
            coeff[2] = ind+16257;
            coeff[3] = ind-1;
            coeff[4] = ind;
            coeff[5] = ind+1;
            coeff[6] = ind+127;
            coeff[7] = ind+128;
            coeff[8] = ind+129;
        }
    }
    else if (YIND == 127){
        if (XIND == 0){
            coeff[0] = 16255;
            coeff[1] = 16128;
            coeff[2] = 16129;
            coeff[3] = 16383;
            coeff[4] = 16256;
            coeff[5] = 16257;
            coeff[6] = 127;
            coeff[7] = 0;
            coeff[8] = 1;
        }
        else if (XIND == 127){
            coeff[0] = 16254;
            coeff[1] = 16255;
            coeff[2] = 16128;
            coeff[3] = 16382;
            coeff[4] = 16383;
            coeff[5] = 0;
            coeff[6] = 126;
            coeff[7] = 127;
            coeff[8] = 16256;
        }
        else{
            coeff[0] = ind-129;
            coeff[1] = ind-128;
            coeff[2] = ind-127;
            coeff[3] = ind-1;
            coeff[4] = ind;
            coeff[5] = ind+1;
            coeff[6] = ind-16257;
            coeff[7] = ind-16256;
            coeff[8] = ind-16255;
        }
    }
    else{
        coeff[0] = ind-129;
        coeff[1] = ind-128;
        coeff[2] = ind-127;
        coeff[3] = ind-1;
        coeff[4] = ind;
        coeff[5] = ind+1;
        coeff[6] = ind+127;
        coeff[7] = ind+128;
        coeff[8] = ind+129;
    }
    return coeff;
}

int print_array(int *arr, int len) {
    int i;
    for (i=0;i<len;i++) {
        std::cout <<"Arr["<<i<<"] = "<<arr[i]<<std::endl;
    }
    return 0;
}

int main() {
    int n;
    while (1) {
        std::cout<<"Enter number 0<=n<=16383(-1 to quit): "<<std::endl;
        std::cin>>n;
        if (n == -1) {
            break;
        }
        print_array(get_param_hardcoded(n), 9);
        print_array(get_param_generic(n), 9);
    }
    return 0;
}