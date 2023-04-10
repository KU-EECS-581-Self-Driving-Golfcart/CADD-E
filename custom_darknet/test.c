#include <stdio.h>
#include <stdlib.h>
#include <math.h>



int main(int argc, char** argv)
{
if (argc != 2) {
printf("Usage: %s <image>\n", argv[0]);
return 1;
}
// Load image
FILE* fp = fopen(argv[1], "rb");
if (!fp) {
    printf("Error: could not open file '%s'\n", argv[1]);
    return 1;
}
unsigned char rgb[MAX_WIDTH * MAX_HEIGHT * 3];
unsigned char gray[MAX_WIDTH * MAX_HEIGHT];
int width, height;
read_jpeg(fp, rgb, &width, &height);
fclose(fp);

// Convert to grayscale
rgb2gray(rgb, gray, width, height);

// Apply Gaussian blur
unsigned char blurred[MAX_WIDTH * MAX_HEIGHT];
gaussianBlur(gray, blurred, width, height);

// Compute gradient
int gradient[MAX_WIDTH * MAX_HEIGHT];
computeGradient(blurred, gradient, width, height);

// Apply non-maximum suppression
unsigned char edges[MAX_WIDTH * MAX_HEIGHT];
nonMaximumSuppression(gradient, edges, width, height);

// Apply hysteresis thresholding
hysteresisThresholding(edges, width, height);

// Find bounding box of edges
BoundingBox box = findBoundingBox(edges, width, height);

// Print bounding box coordinates
printf("Top left: (%d, %d)\n", box.x, box.y);
printf("Bottom right: (%d, %d)\n", box.x + box.width, box.y + box.height);

return 0;
}

