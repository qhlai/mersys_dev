// Useful defines
#define COUTERROR "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << ": "
#define COUTFATAL "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << ": "
#define COUTWARN "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": "
#define COUTNOTICE "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << ": "
#define DEPRECATED_FUNCTION {cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Use of deprecated function" << std::endl; exit(-1);}
#define NOT_IMPLEMENTED {std::cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": This functionality is not implemented" << std::endl; }


#define REDL "\033[1;31m"
#define REDR "\033[0m"
#define COUTRED(...) REDL #__VA_ARGS__ REDR
#define PURPLEL "\033[1;34m"
#define PURPLER "\033[0m"
#define COUTPURPLE(...) PURPLEL #__VA_ARGS__ PURPLER
