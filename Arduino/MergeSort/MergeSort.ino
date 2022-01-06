void setup() {
  Serial.begin(115200);
  float myArray[6][5] = {{2.8,7.1,3.4, 2.0,9.8},{4.6,0.2,8.8,8.1,7.2},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
  float* ptr = &myArray[1][0];
  while(!Serial);
  float Median = MedianFilter(ptr);
  Serial.println(Median);
  delete ptr;
}

void loop() {
  // put your main code here, to run repeatedly:

}

float MedianFilter(float* ptr){
  float newArray[5];
  for (int i = 0 ; i < 5; i++){
    newArray[i] = *(ptr+i);  
  }
  
  mergeSort(newArray,0,4);
  for (int i = 0 ; i < 5; i++){
    Serial.println(newArray[i]);  
  }
  return newArray[2];
}

void merge(float myarray[], int const left, int const mid, int const right)
{
    auto const subArrayOne = mid - left + 1;
    auto const subArrayTwo = right - mid;

    // Create temp arrays
    auto *leftArray = new float[subArrayOne],
         *rightArray = new float[subArrayTwo];

    // Copy data to temp arrays leftArray[] and rightArray[]
    for (auto i = 0; i < subArrayOne; i++)
        leftArray[i] = myarray[left + i];
    for (auto j = 0; j < subArrayTwo; j++)
        rightArray[j] = myarray[mid + 1 + j];

    auto indexOfSubArrayOne = 0, // Initial index of first sub-array
        indexOfSubArrayTwo = 0; // Initial index of second sub-array
    int indexOfMergedArray = left; // Initial index of merged array

    // Merge the temp arrays back into array[left..right]
    while (indexOfSubArrayOne < subArrayOne && indexOfSubArrayTwo < subArrayTwo) {
        if (leftArray[indexOfSubArrayOne] <= rightArray[indexOfSubArrayTwo]) {
            myarray[indexOfMergedArray] = leftArray[indexOfSubArrayOne];
            indexOfSubArrayOne++;
        }
        else {
            myarray[indexOfMergedArray] = rightArray[indexOfSubArrayTwo];
            indexOfSubArrayTwo++;
        }
        indexOfMergedArray++;
    }
    // Copy the remaining elements of
    // left[], if there are any
    while (indexOfSubArrayOne < subArrayOne) {
        myarray[indexOfMergedArray] = leftArray[indexOfSubArrayOne];
        indexOfSubArrayOne++;
        indexOfMergedArray++;
    }
    // Copy the remaining elements of
    // right[], if there are any
    while (indexOfSubArrayTwo < subArrayTwo) {
        myarray[indexOfMergedArray] = rightArray[indexOfSubArrayTwo];
        indexOfSubArrayTwo++;
        indexOfMergedArray++;
    }
}

// begin is for left index and end is
// right index of the sub-array
// of arr to be sorted */
void mergeSort(float myarray[], int const firstEl, int const lastEl)
{
    if (firstEl >= lastEl)
        return; // Returns recursively

    auto mid = firstEl + (lastEl - firstEl) / 2;
    mergeSort(myarray, firstEl, mid);
    mergeSort(myarray, mid + 1, lastEl);
    merge(myarray, firstEl, mid, lastEl);
}
