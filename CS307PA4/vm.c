#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vm_dbg.h"

#define NOPS (16)

#define OPC(i) ((i) >> 12)
#define DR(i) (((i) >> 9) & 0x7)
#define SR1(i) (((i) >> 6) & 0x7)
#define SR2(i) ((i) & 0x7)
#define FIMM(i) ((i >> 5) & 01)
#define IMM(i) ((i) & 0x1F)
#define SEXTIMM(i) sext(IMM(i), 5)
#define FCND(i) (((i) >> 9) & 0x7)
#define POFF(i) sext((i) & 0x3F, 6)
#define POFF9(i) sext((i) & 0x1FF, 9)
#define POFF11(i) sext((i) & 0x7FF, 11)
#define FL(i) (((i) >> 11) & 1)
#define BR(i) (((i) >> 6) & 0x7)
#define TRP(i) ((i) & 0xFF)

/* New OS declarations */

// OS bookkeeping constants
#define PAGE_SIZE       (4096)  // Page size in bytes
#define OS_MEM_SIZE     (2)     // OS Region size. Also the start of the page tables' page
#define Cur_Proc_ID     (0)     // id of the current process
#define Proc_Count      (1)     // total number of processes, including ones that finished executing.
#define OS_STATUS       (2)     // Bit 0 shows whether the PCB list is full or not
#define OS_FREE_BITMAP  (3)     // Bitmap for free pages

// Process list and PCB related constants
#define PCB_SIZE  (3)  // Number of fields in a PCB
#define PID_PCB   (0)  // Holds the pid for a process
#define PC_PCB    (1)  // Value of the program counter for the process
#define PTBR_PCB  (2)  // Page table base register for the process

#define CODE_SIZE       (2)  // Number of pages for the code segment
#define HEAP_INIT_SIZE  (2)  // Number of pages for the heap segment initially

bool running = true;

typedef void (*op_ex_f)(uint16_t i);
typedef void (*trp_ex_f)();

enum { trp_offset = 0x20 };
enum regist { R0 = 0, R1, R2, R3, R4, R5, R6, R7, RPC, RCND, PTBR, RCNT };
enum flags { FP = 1 << 0, FZ = 1 << 1, FN = 1 << 2 };

uint16_t mem[UINT16_MAX] = {0};
uint16_t reg[RCNT] = {0};
uint16_t PC_START = 0x3000;

void initOS();
int createProc(char *fname, char *hname);
void loadProc(uint16_t pid);
uint16_t allocMem(uint16_t ptbr, uint16_t vpn, uint16_t read, uint16_t write);  // Can use 'bool' instead
int freeMem(uint16_t ptr, uint16_t ptbr);
static inline uint16_t mr(uint16_t address);
static inline void mw(uint16_t address, uint16_t val);
static inline void tbrk();
static inline void thalt();
static inline void tyld();
static inline void trap(uint16_t i);

static inline uint16_t sext(uint16_t n, int b) { return ((n >> (b - 1)) & 1) ? (n | (0xFFFF << b)) : n; }
static inline void uf(enum regist r) {
    if (reg[r] == 0)
        reg[RCND] = FZ;
    else if (reg[r] >> 15)
        reg[RCND] = FN;
    else
        reg[RCND] = FP;
}
static inline void add(uint16_t i)  { reg[DR(i)] = reg[SR1(i)] + (FIMM(i) ? SEXTIMM(i) : reg[SR2(i)]); uf(DR(i)); }
static inline void and(uint16_t i)  { reg[DR(i)] = reg[SR1(i)] & (FIMM(i) ? SEXTIMM(i) : reg[SR2(i)]); uf(DR(i)); }
static inline void ldi(uint16_t i)  { reg[DR(i)] = mr(mr(reg[RPC]+POFF9(i))); uf(DR(i)); }
static inline void not(uint16_t i)  { reg[DR(i)]=~reg[SR1(i)]; uf(DR(i)); }
static inline void br(uint16_t i)   { if (reg[RCND] & FCND(i)) { reg[RPC] += POFF9(i); } }
static inline void jsr(uint16_t i)  { reg[R7] = reg[RPC]; reg[RPC] = (FL(i)) ? reg[RPC] + POFF11(i) : reg[BR(i)]; }
static inline void jmp(uint16_t i)  { reg[RPC] = reg[BR(i)]; }
static inline void ld(uint16_t i)   { reg[DR(i)] = mr(reg[RPC] + POFF9(i)); uf(DR(i)); }
static inline void ldr(uint16_t i)  { reg[DR(i)] = mr(reg[SR1(i)] + POFF(i)); uf(DR(i)); }
static inline void lea(uint16_t i)  { reg[DR(i)] =reg[RPC] + POFF9(i); uf(DR(i)); }
static inline void st(uint16_t i)   { mw(reg[RPC] + POFF9(i), reg[DR(i)]); }
static inline void sti(uint16_t i)  { mw(mr(reg[RPC] + POFF9(i)), reg[DR(i)]); }
static inline void str(uint16_t i)  { mw(reg[SR1(i)] + POFF(i), reg[DR(i)]); }
static inline void rti(uint16_t i)  {} // unused
static inline void res(uint16_t i)  {} // unused
static inline void tgetc()        { reg[R0] = getchar(); }
static inline void tout()         { fprintf(stdout, "%c", (char)reg[R0]); }
static inline void tputs() {
  uint16_t *p = mem + reg[R0];
  while(*p) {
    fprintf(stdout, "%c", (char) *p);
    p++;
  }
}
static inline void tin()      { reg[R0] = getchar(); fprintf(stdout, "%c", reg[R0]); }
static inline void tputsp()   { /* Not Implemented */ }
static inline void tinu16()   { fscanf(stdin, "%hu", &reg[R0]); }
static inline void toutu16()  { fprintf(stdout, "%hu\n", reg[R0]); }

trp_ex_f trp_ex[10] = {tgetc, tout, tputs, tin, tputsp, thalt, tinu16, toutu16, tyld, tbrk};
static inline void trap(uint16_t i) { trp_ex[TRP(i) - trp_offset](); }
op_ex_f op_ex[NOPS] = {/*0*/ br, add, ld, st, jsr, and, ldr, str, rti, not, ldi, sti, jmp, res, lea, trap};

/**
  * Load an image file into memory.
  * @param fname the name of the file to load
  * @param offsets the offsets into memory to load the file
  * @param size the size of the file to load
*/
void ld_img(char *fname, uint16_t *offsets, uint16_t size) {
    FILE *in = fopen(fname, "rb");
    if (NULL == in) {
        fprintf(stderr, "Cannot open file %s.\n", fname);
        exit(1);
    }

    for (uint16_t s = 0; s < size; s += PAGE_SIZE) {
        uint16_t *p = mem + offsets[s / PAGE_SIZE];
        uint16_t writeSize = (size - s) > PAGE_SIZE ? PAGE_SIZE : (size - s);
        fread(p, sizeof(uint16_t), (writeSize), in);
    }
    
    fclose(in);
}

void run(char *code, char *heap) {
  while (running) {
    uint16_t i = mr(reg[RPC]++);
    op_ex[OPC(i)](i);
  }
}

// YOUR CODE STARTS HERE

void initOS() {

  //setting the system variables

  mem[Cur_Proc_ID] = 0xFFFF; //this means there is no process running
  mem[Proc_Count] = 0; //this means there is no process created yet
  mem[OS_STATUS] = 0x0000; //initializing the os status to 0x0000 as described in the homework document

  //initializing the bitmap for keeping the free pages

  mem[OS_FREE_BITMAP] = 0x1FFF; //basically this marks the first two pages allocated
  mem[4] = 0xFFFF; // This part is also mentioned in the homework document, it states that bitmap is 32 bytes hence we need one more additional mem[] container


  //now we need to mark the first 2 pages (OS Region) as allocated
  /*
  for(int i = 0; i < OS_MEM_SIZE; i++){ // OS_Mem_size = 2 hence we mark the first 2 pages as allocated
    mem[OS_FREE_BITMAP] &= ~(1 << i);//set the i'th bit to 0
  }
  */
  
  /*
  here is how this part works: 
  mem[OS_FREE_BITMAP] &= ~(1 << i);//set the i'th bit to 0

  initially i = 0
  1 << 0 = 0000 0000 0000 0001
  ~(1 << 0) = 1111 1111 1111 1110
  and if you do the and operation it is clear that the first bit is set to 0 and the rest are all 1
  same idea for i = 1
  */

}

int createProc(char *fname, char *hname) {
    if (mem[OS_STATUS] & 1) {
        // OS region is full, cannot create a new PCB
        printf("The OS memory region is full. Cannot create a new PCB.\n");
        return 0; // Failure
    }

    uint16_t maxPID = 0; 
    uint16_t procCount = mem[Proc_Count];

    // Find the maximum PID so far to assign a new unique PID
    for (uint16_t i = 0; i < procCount; i++) {
        uint16_t pcbStart = 12 + (i * PCB_SIZE);
        uint16_t pid = mem[pcbStart + PID_PCB];
        if (pid != 0xFFFF && pid > maxPID) {
            maxPID = pid;
        }
    }

    uint16_t newPID = (procCount == 0) ? 0 : maxPID + 1;
    uint16_t pcbStart = 12 + (procCount * PCB_SIZE);

    // Check if we can place another PCB in OS region
    if (pcbStart + PCB_SIZE > (PAGE_SIZE * OS_MEM_SIZE / 2)) {
        printf("The OS memory region is full. Cannot create a new PCB.\n");
        return 0; // Failure
    }

    // Fill in PCB fields
    mem[pcbStart + PID_PCB] = newPID;   // Set PID
    mem[pcbStart + PC_PCB] = 0x3000;    // Default PC start
    static uint16_t nextPTBR = 0x1000;  // Start page tables at 0x1000
    uint16_t ptbr = nextPTBR;
    nextPTBR += 32; // Each page table is 32 entries (64 bytes)(I am not sure but I might need to do nextPTBR += 64)

    mem[pcbStart + PTBR_PCB] = ptbr;

    // Allocate code segment: 2 pages at VPN 6 and 7
    // Code segment is read-only (set write = 0)
    for (uint16_t i = 0; i < CODE_SIZE; i++) {
        uint16_t vpn = 6 + i; 
        if (allocMem(ptbr, vpn, UINT16_MAX, 0) == 0) {
            printf("Cannot create code segment.\n");
            // Free already allocated code pages
            for (uint16_t j = 0; j < i; j++) {
                freeMem(6 + j, ptbr);
            }
            return 0; // Failure
        }
    }

    // Compute the physical addresses for the code pages and load code
    uint16_t codeOffsets[CODE_SIZE];
    for (uint16_t i = 0; i < CODE_SIZE; i++) {
        uint16_t vpn = 6 + i;
        uint16_t pte = mem[ptbr + vpn];
        uint16_t pfn = pte >> 11;
        codeOffsets[i] = pfn << 11; // Physical address of the allocated code page
    }
    ld_img(fname, codeOffsets, CODE_SIZE * PAGE_SIZE);

    // Allocate heap segment: 2 pages at VPN 8 and 9 (initial heap)
    for (uint16_t i = 0; i < HEAP_INIT_SIZE; i++) {
        uint16_t heapVPN = 8 + i;
        if (allocMem(ptbr, heapVPN, UINT16_MAX, UINT16_MAX) == 0) {
            printf("Cannot create heap segment.\n");
            // Free allocated code pages
            for (uint16_t j = 0; j < CODE_SIZE; j++) {
                freeMem(6 + j, ptbr);
            }
            // Free already allocated heap pages
            for (uint16_t j = 0; j < i; j++) {
                freeMem(8 + j, ptbr);
            }
            return 0; // Failure
        }
    }

    // Compute the physical addresses for the heap pages and load heap
    uint16_t heapOffsets[HEAP_INIT_SIZE];
    for (uint16_t i = 0; i < HEAP_INIT_SIZE; i++) {
        uint16_t vpn = 8 + i;
        uint16_t pte = mem[ptbr + vpn];
        uint16_t pfn = pte >> 11;
        heapOffsets[i] = pfn << 11; // Physical address of the allocated heap page
    }
    ld_img(hname, heapOffsets, HEAP_INIT_SIZE * PAGE_SIZE);

    // Process created successfully
    mem[Proc_Count] += 1;

    // Update OS_STATUS if PCB region is full
    uint16_t maxPCBs = ((PAGE_SIZE * OS_MEM_SIZE / 2) - 12) / PCB_SIZE;
    if (mem[Proc_Count] >= maxPCBs) {
        mem[OS_STATUS] |= 1;  // Set LSB
    } else {
        mem[OS_STATUS] &= ~1; // Clear LSB
    }

    return 1; // Success
}


void loadProc(uint16_t pid) {
  //the aim of this function is to load a process into the cpu registers
  //uint16_t pcbStart = (OS_MEM_SIZE * PAGE_SIZE / 2) + (pid * PCB_SIZE);
  uint16_t pcbStart = 12 + (pid * PCB_SIZE);

  /*
  OS_MEM_SIZE => is the size of the OS region in pages(in the document it is stated as 2 pages)
  PAGE_SIZE => is the size of each page (4KB, it is also stated in the homework document)
  hence OS_MEM_SIZE * PAGE_SIZE is the total size of the OS region in bytes (2 * 4096 = 8192 bytes)

  /2: The VM is not byte addressed it is word addressed
  Each memory location represents a 16 bit value which is 2 bytes
  hence when we divide it by 2 we effectively convert it into words (the unit used by the VM memory)
  this part was specified in the homework document as VM is not byte-addressed. Its elements are of type uint16_t (unsigned 16-bit integers) and they occupy 2 bytes

  pid * PCB_SIZE => This is the number of words used for each PCB which is 3
  so to find the PCB we multiply it by pid to locate the offset from the start of the PCB list
  */

  //loading the values froom the PCB to the cpu registers
  //Process Controll Block stores PC(program counter) and Page Table Base Register(PTBR)
  reg[RPC] = mem[pcbStart + PC_PCB]; //loading the program counter
  reg[PTBR] = mem[pcbStart + PTBR_PCB]; //loading the page table base register

  mem[Cur_Proc_ID] = pid; //the current process id is pid(also stated in the homework document)

}
/*
uint16_t allocMem(uint16_t ptbr, uint16_t vpn, uint16_t read, uint16_t write) {
  int freePage = -1; 
  //we initialize the free page to -1 because if at the end of search we 
  //still have -1 value in the freePage variables then it means we couldn't finda correct spot

  //we go through all the pages to find a free spot
  //we go until 32 since in the homework document it is stated that there are 32 pages 
  for(int i = 0; i  < 32; i++){
    if((mem[OS_FREE_BITMAP] >> i) & 1){
      freePage = i;//checking if the i'th bit is free
      break;//found a free page hence we can stop
    }
  }

  if(freePage == -1){
    //if we are here it means that we couldn't find a free page, hence we return 0 as stated in the homework document
    return 0;
  }

  uint16_t *pageTable = mem + ptbr;//this is a pointe arithmetic, basically finding the base of the page table
  uint16_t pte = pageTable[vpn]; //pte is a 16-bit value that holds the metadata for a specific virtual page number(VPN)
  //basically each vpn corresponds to a pte which maps it to a physical frame number(PFN) 

  if(pte & 1){
    //pte & 1 get's the least significant bit(rightmost bit) 
    //if the LSB is 1 it means the page is allocated
    //if the LSB is 0 it means the page is not allocated

    return 0; //VPN is already allocated hence allocation failed
  }

  //if we are here we have found a page and we are ready to allocate the page
  mem[OS_FREE_BITMAP] &= ~(1 << freePage); //marking the page as allocated by setting the freePage'th bit to 0
  pageTable[vpn] = (freePage << 11) | ((read == UINT16_MAX ? 1 : 0) << 1) | ((write == UINT16_MAX ? 1 : 0) << 2) | 1;//it shouldn't be 8 it should be 11
  //there is a shorter approach but I am not sure if I can do this: 
  //pageTable[vpn] = (freePage << 11) | (read << 1) | (write << 2) | 1;//it shouldn't be 8 it should be 11

  //freePage << 11 => PFN occupies the left most 5 significant bits hence we shift 11 bits to the left
  // read occupies the second rightmost bit
  //write occupies the third rightmost bit
  //1 is the rightmost bit which stands for it is valid

  return 1;


}
*/
uint16_t allocMem(uint16_t ptbr, uint16_t vpn, uint16_t read, uint16_t write) {

  int freePage = -1; // Initialize to -1 to indicate no page found

  // Iterate over all 32 pages, prioritizing MSB to LSB order
  for (int i = 0; i < 32; i++) {
    uint16_t bitmap = (i < 16) ? mem[OS_FREE_BITMAP] : mem[4]; // Choose the correct bitmap
    int bitIndex = (i < 16) ? 15 - i : 31 - i;                 // Calculate bit index (MSB to LSB for each bitmap)

    if ((bitmap >> bitIndex) & 1) { // Check if the bit at the index is free
        freePage = i;               // Found a free page
        break;                      // Exit the loop
    }
  }

  // If no free page was found, return 0 (allocation failed)
  if (freePage == -1) {
    return 0;
  }

  // Find the page table using the provided `ptbr`
  uint16_t *pageTable = mem + ptbr;

  // Check if the given VPN is already allocated
  uint16_t pte = pageTable[vpn];
  if (pte & 1) {
    // LSB = 1 means the VPN is already allocated
    return 0;
  }

  // Debugging: Print the bitmap status before allocation
  //printf("Before allocation:\n");
  //printf("mem[3] = 0x%04X (dec: %d)\n", mem[3], mem[3]);
  //printf("mem[4] = 0x%04X (dec: %d)\n", mem[4], mem[4]);
  //printf("freePage = %d, bitIndex = %d\n", freePage, (freePage < 16) ? 15 - freePage : 31 - freePage);

  // Mark the free page as allocated in the appropriate bitmap
  if (freePage < 16) {
    mem[OS_FREE_BITMAP] &= ~(1 << (15 - freePage)); // Clear the corresponding bit in `OS_FREE_BITMAP`
  } else {
    mem[4] &= ~(1 << (31 - freePage));             // Clear the corresponding bit in `mem[4]`
  }

  // Debugging: Print the bitmap status after allocation
  //printf("After allocation:\n");
  //printf("mem[3] = 0x%04X (dec: %d)\n", mem[3], mem[3]);
  //printf("mem[4] = 0x%04X (dec: %d)\n", mem[4], mem[4]);
  //printf("PTBR: 0x%04X, Writing PTE for VPN %d at address 0x%04X\n",ptbr, vpn, ptbr + vpn);

  // Populate the PTE with the allocated page information
  pageTable[vpn] = (freePage << 11) |                // PFN (leftmost 5 bits)
                    ((read == UINT16_MAX ? 1 : 0) << 1) | // Read bit (2nd LSB)
                    ((write == UINT16_MAX ? 1 : 0) << 2) | // Write bit (3rd LSB)
                    1;                                // Valid bit (LSB)

  return 1; // Allocation successful
}

/*
int freeMem(uint16_t vpn, uint16_t ptbr) {
  //this function is the dual of allocMem function, so the implementation is very similar
  uint16_t *pageTable = mem + ptbr;//locating the page table(again using a basic pointer arithmetic)

  uint16_t pte = pageTable[vpn];//access the PTE by using the vpn

  if(!(pte & 1)){
    //if we are here it means pte & 1 = 0 meaning the page is already freed hence return 0
    return 0;
  }

  uint16_t pfn = pte >> 11; //again we are doing the opposite of what we did in line 213, we are now extracting the pfn

  mem[OS_FREE_BITMAP] |= (1 << pfn);//setting the pfn bit to 1 meaning it is free

  pageTable[vpn] &= ~1; // setting the valid bit to 0
  //the reasoning is as follows: 
  // ~1 = ~(00000000....0001)
  // => ~(11111111....1110)
  //hence we are only setting the LSB(which is the valid bit) to 0

  return 1;//function successfully terminated

}
*/
int freeMem(uint16_t vpn, uint16_t ptbr) {
  uint16_t *pageTable = mem + ptbr; // Locate the page table
  uint16_t pte = pageTable[vpn];    // Access the PTE using the VPN

  if (!(pte & 1)) {
    // If the valid bit is 0, the page is already free
    return 0;
  }

  uint16_t pfn = pte >> 11; // Extract the PFN from the PTE

  // Mark the page as free in the appropriate bitmap
  if (pfn < 16) {
    mem[OS_FREE_BITMAP] |= (1 << (15 - pfn)); // Set the MSB-to-LSB bit in `OS_FREE_BITMAP`
  } else {
    mem[4] |= (1 << (31 - pfn));  // Set the MSB-to-LSB bit in `mem[4]`
  }

  pageTable[vpn] &= ~1;  // Clear the valid bit in the PTE

  return 1;  // Success
}


static inline void tbrk() {
  uint16_t r0 = reg[R0]; // getting the value of R0 register
  //uint16_t vpn = r0 & 0x1F;  // Extract VPN (first 5 bits)
  //0xF800
  uint16_t vpn = (r0 >> 11) & 0x1F;  // Extract VPN from the most significant 5 bits
  // 0x1F = 0001 1111, hence when we do the AND operation we effectively extract the first 5 bits
  bool write = (r0 >> 2) & 1;  // Extract write bit (specified in the homework doc)
  bool read = (r0 >> 1) & 1;   // Extract read bit (specified in the homework doc)
  bool allocate = r0 & 1;      // Extract allocation/freeing bit (specified in the homework doc)
  uint16_t curProcID = mem[Cur_Proc_ID]; // Get the current process ID

  if (curProcID == 0xFFFF) {
      // No process is currently running
      printf("No process is currently running.\n");
      return;
  }

  uint16_t ptbr = reg[PTBR]; // Getting the page table base register

  // Convert the boolean read/write to UINT16_MAX or 0 for allocMem
  uint16_t read_val = read ? UINT16_MAX : 0;
  uint16_t write_val = write ? UINT16_MAX : 0;

  if (allocate) {
    // Allocation request
    printf("Heap increase requested by process %d.\n", curProcID);
    //printf("line(486)the value of reg[PTBR] = %d\n", ptbr);

    uint16_t status = allocMem(ptbr, vpn, read_val, write_val); // Attempt allocation

    if (status == 0) {
      // Allocation failed
      if ((mem[OS_FREE_BITMAP] & 1) == 0) {
        // No free pages left
        printf("Cannot allocate more space for pid %d since there is no free page frames.\n", curProcID);
      } else {
        // Page already allocated
        printf("Cannot allocate memory for page %d of pid %d since it is already allocated.\n", vpn, curProcID);
      }
    }

  } else {
    // Deallocation request
    printf("Heap decrease requested by process %d.\n", curProcID);

    int status = freeMem(vpn, ptbr); // Attempt to free the page
    if (status == 0) {
      // Already deallocated
      printf("Cannot free memory of page %d of pid %d since it is not allocated.\n", vpn, curProcID);
    }
  }
}

static inline void tyld() {
  uint16_t currentPID = mem[Cur_Proc_ID]; // Get the currently running process ID

  if (currentPID == 0xFFFF) {
      // No processes running currently
      return;
  }

  uint16_t procCount = mem[Proc_Count];
  uint16_t nextPID = (currentPID + 1) % procCount;

  // Find the next runnable process
  while (nextPID != currentPID) {
      uint16_t nextPCBStart = 12 + (nextPID * PCB_SIZE);
      if (mem[nextPCBStart + PID_PCB] != 0xFFFF) {
          // Found a runnable process
          break;
      }
      nextPID = (nextPID + 1) % procCount;  
  }

  if (nextPID == currentPID) {
      // No other runnable process was found, so no context switch happens.
      // Return without modifying the PCB.
      return;
  }

  // At this point, we have a different process to switch to, so save the current process state
  uint16_t pcbStart = 12 + (currentPID * PCB_SIZE);
  mem[pcbStart + PC_PCB] = reg[RPC];      // Save PC
  mem[pcbStart + PTBR_PCB] = reg[PTBR];   // Save PTBR

  // Load the new process state
  uint16_t nextPCBStart = 12 + (nextPID * PCB_SIZE);
  reg[RPC] = mem[nextPCBStart + PC_PCB];  // Restore PC
  reg[PTBR] = mem[nextPCBStart + PTBR_PCB]; // Restore PTBR
  mem[Cur_Proc_ID] = nextPID;             // Update current process ID

  // Print the context switch message only if we actually switched processes
  printf("We are switching from process %d to %d.\n", currentPID, nextPID);
}


// Instructions to modify
// Instructions to modify
static inline void thalt() {
  uint16_t curProcID = mem[Cur_Proc_ID]; // Getting the current process ID

  if (curProcID == 0xFFFF) {
    // If we are here it means there are not processes  running currently hence we set running false  and exit the function
    running = false;
    return;
  } 

  //we need to free the pages we have allocated(as described in the homework document)
  uint16_t ptbr = reg[PTBR]; //getting the page table register
  for(uint16_t i = 0; i< 32; i++){
    freeMem(i, ptbr);//freeing all possible pages
  }

  //now that we hav freed the pages we need to mark the current process's PCB as free
  uint16_t pcbStart = 12 + (curProcID * PCB_SIZE);
  mem[pcbStart + PID_PCB] = 0xFFFF; // Mark the PID as freed, this leaves a footprint in memory just like it was described in the homework document

   // Find the next runnable process
  uint16_t procCount = mem[Proc_Count]; // Total number of processes
  uint16_t nextPID = (curProcID + 1) % procCount;

  //now we need to find the next process
  while (nextPID != curProcID) {
      uint16_t nextPCBStart = 12 + (nextPID * PCB_SIZE);
      if (mem[nextPCBStart + PID_PCB] != 0xFFFF) {
          // Found a runnable process
          break;
      }
      nextPID = (nextPID + 1) % procCount; // Try the next PID
  }

  if (nextPID == curProcID) {
    // if we are here it means the current process was the final process hence we set running to false
    running = false;
    return;
  }

  // doing the same stuff, we update the cpu registers for the next process
  uint16_t nextPCBStart = 12 + (nextPID * PCB_SIZE);
  reg[RPC] = mem[nextPCBStart + PC_PCB];  // Restore the PC
  reg[PTBR] = mem[nextPCBStart + PTBR_PCB];  // Restore the PTBR
  mem[Cur_Proc_ID] = nextPID;  // Update the current process ID

}


static inline uint16_t mr(uint16_t address) {
  //the idea of the bitwise manipulations were explained before
  uint16_t vpn = (address >> 11) & 0x1F; // Extract VPN (first 5 bits)
  uint16_t offset = address & 0x7FF;    // Extract offset (last 11 bits)

  if (vpn < OS_MEM_SIZE) {
        // if we are here it means vpn is in the reserved region, hence we must throw a segmentation fault error
        printf("Segmentation fault.\n");
        exit(1);//exit the program
  }

  uint16_t ptbr = reg[PTBR]; // Get the page table base register
  uint16_t *pageTable = mem + ptbr; // Base of the page table
  uint16_t pte = pageTable[vpn]; // Fetch the PTE for the given VPN

  if (!(pte & 1)) {
    // if we are here it means the valid bit is 0 hence it is not valid
    printf("Segmentation fault inside free space.\n");
    exit(1);//exit the program
  }

  uint16_t pfn = pte >> 11; // Extract PFN (most significant 5 bits of PTE)
  uint16_t physicalAddress = (pfn << 11) | offset; // Combine PFN and offset(as described int the homework document)

  return mem[physicalAddress]; // Return the value at the physical address
}

static inline void mw(uint16_t address, uint16_t val) {
  uint16_t vpn = (address >> 11) & 0x1F; // Extract VPN (first 5 bits)
  uint16_t offset = address & 0x7FF;    // Extract offset (last 11 bits)

  if (vpn < OS_MEM_SIZE) {
      // Check if the VPN belongs to the reserved region
      printf("Segmentation fault.\n");
      exit(1);//exit the program
  }

  uint16_t ptbr = reg[PTBR]; // Get the page table base register
  uint16_t *pageTable = mem + ptbr; // Base of the page table
  uint16_t pte = pageTable[vpn]; // Fetch the PTE for the given VPN

  if (!(pte & 1)) {
      // if we are here it means the valid bit is 0 hence it is a segmentation fault
      printf("Segmentation fault inside free space.\n");
      exit(1);//we should exit the program
  }

  if (!(pte & (1 << 2))) {
      // if we are here it means the write bit is 0 hence we can not write to this page
      printf("Cannot write to a read-only page.\n");
      exit(1);
  }

  uint16_t pfn = pte >> 11; // Extract PFN (most significant 5 bits of PTE)
  uint16_t physicalAddress = (pfn << 11) | offset; // Combine PFN and offset(as described in the homework document)

  mem[physicalAddress] = val; // Write the value to the physical address
}

// YOUR CODE ENDS HERE
