/*****************************************************************************
* Copyright (c) 2019 - Hemant Sharma - All Rights Reserved
*
* Feel free to use this Code at your own risk for your own purposes.
*
*******************************************************************************/
/******************************************************************************
* Title:		Memory source file
* Filename:		dri_mem.cpp
* Author:		HS
* Origin Date:	10/04/2019
* Version:		1.0.0
* Notes:
*
* Change History
* --------------
*
*******************************************************************************/

/** @file:	dri_mem.cpp
 *  @brief:	This source file contains memory operators functions definitions
 */


/******************************************************************************
* Includes
*******************************************************************************/
#include <dri_mem.h>


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Definitions
*******************************************************************************/

/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
int memcmp( const void * p_arg_one, const void *p_arg_two, size_t size )
{
	/* Local variables */
	const unsigned char *p_arg_one_l = (unsigned char *)p_arg_one;
	const unsigned char *p_arg_two_l = (unsigned char *)p_arg_two;
	size_t size_l = 0;
	int ret_l = 0;

	/* Compare memory for differences */
	for( size_l = 0; size_l < size; ++size_l )
	{
		/* Values are not equal */
		if( *(p_arg_one_l + size_l) != *(p_arg_two_l + size_l) )
		{
			/* Check the greater or smaller between both */
			ret_l = ( *(p_arg_one_l + size_l) - *(p_arg_two_l + size_l) );
		}
		else/* Values are equal */
		{
			/* Do not do anything */
		}
	}

	/* Check for return value */
	return (ret_l != 0 ? ret_l : 0);
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>	
 *
 */
void *memcpy( void *__restrict p_dest, const void *__restrict p_src,
				size_t size )
{
#define PREFER_SIZE_OVER_SPEED
#if defined(PREFER_SIZE_OVER_SPEED) || defined(__OPTIMIZE_SIZE__)
	/* Local variables */
	char * p_dest_l = (char *) p_dest;
	const char * p_src_l = (const char *) p_src;
	size_t size_l = 0;

	for( size_l = 0; size_l < size; ++size_l )
	{
		*p_dest_l++ = *p_src_l++;
	}

	return p_dest;
#else
    /* Taken from Newlib Source */
	/* Nonzero if either X or Y is not aligned on a "long" boundary.  */
	#define UNALIGNED(X, Y) \
		(((long)X & (sizeof (long) - 1)) | ((long)Y & (sizeof (long) - 1)))

	/* How many bytes are copied each iteration of the 4X unrolled loop.  */
	#define BIGBLOCKSIZE    (sizeof (long) << 2)

	/* How many bytes are copied each iteration of the word copy loop.  */
	#define LITTLEBLOCKSIZE (sizeof (long))

	/* Threshhold for punting to the byte copier.  */
	#define TOO_SMALL(LEN)  ((LEN) < BIGBLOCKSIZE)

	char *dst = (char *)p_dest;
	const char *src = (const char *)p_src;
	long *aligned_dst;
	const long *aligned_src;

	/* If the size is small, or either SRC or DST is unaligned,
     then punt into the byte copy loop.  This should be rare.  */
	if (!TOO_SMALL(size) && !UNALIGNED (src, dst))
	{
		aligned_dst = (long*)dst;
		aligned_src = (long*)src;

		/* Copy 4X long words at a time if possible.  */
		while (size >= BIGBLOCKSIZE)
		{
			*aligned_dst++ = *aligned_src++;
			*aligned_dst++ = *aligned_src++;
			*aligned_dst++ = *aligned_src++;
			*aligned_dst++ = *aligned_src++;
			size -= BIGBLOCKSIZE;
		}

		/* Copy one long word at a time if possible.  */
		while (size >= LITTLEBLOCKSIZE)
		{
			*aligned_dst++ = *aligned_src++;
			size -= LITTLEBLOCKSIZE;
		}

		/* Pick up any residual with a byte copier.  */
		dst = (char*)aligned_dst;
		src = (char*)aligned_src;
	}

	while (size--)
	{
		*dst++ = *src++;
	}

	/* Undefine macros */
	#undef UNALIGNED
	#undef BIGBLOCKSIZE
	#undef LITTLEBLOCKSIZE
	#undef TOO_SMALL

	return p_dest;
#endif /* not PREFER_SIZE_OVER_SPEED */
#ifdef PREFER_SIZE_OVER_SPEED
#undef PREFER_SIZE_OVER_SPEED
#endif
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
void *memset( void * p_arg, int setval, size_t size )
{
	/* Local variables */
	char * p_arg_l = (char *) p_arg;

	/* Fill values in memory with setval */
	while( size-- )
	{
		*p_arg_l++ = (char) setval;
	}

	/* Return starting memory address */
	return p_arg;
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
void *realloc( void * p_arg, size_t size )
{
	/* Local variables */
    (void) p_arg;
    (void) size;
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
void *calloc( size_t nmemb , size_t size )
{
	/* Local Variables */
	void * p_mem = NULL;
	size_t length = 0;

	/* Check for memory allocation */
	p_mem = (0 == nmemb || 0 ==size) ? NULL : pvPortMalloc( nmemb * size );

	/* Check memory validity and set it to Zero */
	if( p_mem != NULL )
	{
		char * p_mem_l = (char *) p_mem;
		for( length = 0; length < nmemb; ++length )
		{
			*p_mem_l = 0;
			++p_mem_l;
		}
	}

	/* Return allocated memory */
	p_mem;
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
void *memalign( size_t alignment, size_t size )
{
	/* Local variables */
    (void) alignment;
    (void) size;
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
int strcmp (const char *p_s1, const char *p_s2)
{
	/* Local variables */
	const char *p_arg_one_l = (char *)p_s2;
	const char *p_arg_two_l = (char *)p_s2;
	size_t size_l = 0;
	int ret_l = 0;

	/* Compare memory for differences */
	for( size_l = 0; (*p_arg_one_l != '\0' || *p_arg_two_l != '\0'); ++size_l )
	{
		/* Values are not equal */
		if( *(p_arg_one_l + size_l) != *(p_arg_two_l + size_l) )
		{
			/* Check the greater or smaller between both */
			ret_l = ( *(p_arg_one_l + size_l) - *(p_arg_two_l + size_l) );
		}
		else/* Values are equal */
		{
			/* Do not do anything */
		}
	}

	/* Check for return value */
	return (ret_l != 0 ? ret_l : 0);
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
int strncmp( const char *p_s1, const char *p_s2, size_t size )
{
	/* Local variables */
	const char *p_arg_one_l = (char *)p_s1;
	const char *p_arg_two_l = (char *)p_s2;
	size_t size_l = 0;
	int ret_l = 0;

	/* Compare memory for differences */
	for( size_l = 0; size_l < size; ++size_l )
	{
		/* Values are not equal */
		if( *(p_arg_one_l + size_l) != *(p_arg_two_l + size_l) )
		{
			/* Check the greater or smaller between both */
			ret_l = ( *(p_arg_one_l + size_l) - *(p_arg_two_l + size_l) );
		}
		else/* Values are equal */
		{
			/* Do not do anything */
		}
	}

	/* Check for return value */
	return (ret_l != 0 ? ret_l : 0);
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
char * strcpy (char * p_dest, const char *p_src)
{
	/* Local variables */
	char * p_dest_l = p_dest;
	const char * p_src_l = p_src;

	/* Start copying data from src to dest */
	while( *p_src_l != '\0' )
	{
		*p_dest_l = *p_src_l;
		/* Increment both src and dest pointers */
		++p_src_l;
		++p_dest_l;
	}
	/* We have reached Null termination */
	*p_dest_l = *p_src_l;

	/* Return destination pointer */
	return p_dest;
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */

char *strncpy(char *p_dest, const char *p_src, size_t n)
{
	/* Local variables */
	char * p_dest_l = p_dest;
	const char * p_src_l = p_src;
	size_t n_l = 0;

	/* Start copying data from src to dest */
	for( n_l = 0; (n_l < n) && (*p_src_l != '\0'); ++n_l )
	{
		*p_dest_l = *p_src_l;
		/* Increment both src and dest pointers */
		++p_src_l;
		++p_dest_l;
	}
	for( ; n_l < n; ++n_l )
	{
		*p_dest_l = '\0';
		++p_src_l;
	}

	/* Return destination pointer */
	return p_dest;
}


/**
 * @function
 *
 * @brief
 *
 * <i>Imp Note:</i>
 *
 */
size_t strlen(const char * p_arg)
{
	/* Local variables */
	char * p_arg_l = (char *) p_arg;
	size_t length_l = 0;

	/* Check for Null termination character */
	for( /* Already Initialized */; *p_arg_l != '\0'; ++p_arg_l )
	{
		length_l += 1;
	}

	/* Return length of string */
	return length_l;
}


/*********************************** End of File ******************************/
