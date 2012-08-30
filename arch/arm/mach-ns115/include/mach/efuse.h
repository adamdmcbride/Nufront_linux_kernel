#if !defined(__EFUSE_H__)
#define __EFUSE_H__

struct EFuse_data {
    union {
        struct {
            unsigned int low;
            unsigned int high;
        } a;
        struct {
            unsigned int cpu_max_speed:3;
            unsigned int leakage_limit:6;
            unsigned int site_no:1;
            unsigned int make_second:6;
            unsigned int make_minute:6;
            unsigned int make_hour:5;
            unsigned int make_day:5;

            unsigned int make_month:4;
            unsigned int make_year:7;
            unsigned int ATE_no:8;
            unsigned int factory_no:3;
            unsigned int unused:10;
        } b;
    };
};

#define NS115_EFUSE_BASE  0x06160000

#define NS115_EFUSE_CEN   	(0x0104)
#define NS115_EFUSE_REQ   	(0x0108)
#define NS115_EFUSE_ADDR  	(0x010c)
#define NS115_EFUSE_DATA_0	(0x0110)
#define NS115_EFUSE_DATA_1  (0x0114)



void init_ns115_efuse_data();
struct EFuse_data* get_ns115_efuse_data();
void dump_efuse_data(struct EFuse_data*);
#endif
