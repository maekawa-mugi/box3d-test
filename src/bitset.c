// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"

#if defined( BOX3D_PS2 )
#include "ps2_mmi.h"
#endif

#include <string.h>

b3BitSet b3CreateBitSet( uint32_t bitCapacity )
{
	b3BitSet bitSet = { 0 };
	bitSet.blockCapacity = ( bitCapacity + sizeof( uint64_t ) * 8 - 1 ) / ( sizeof( uint64_t ) * 8 );
	bitSet.blockCount = 0;
	bitSet.bits = (uint64_t*)b3Alloc( bitSet.blockCapacity * sizeof( uint64_t ) );
#if defined( BOX3D_PS2 )
	b3Ps2ClearBytes( bitSet.bits, bitSet.blockCapacity * sizeof( uint64_t ) );
#else
	memset( bitSet.bits, 0, bitSet.blockCapacity * sizeof( uint64_t ) );
#endif
	return bitSet;
}

void b3DestroyBitSet( b3BitSet* bitSet )
{
	b3Free( bitSet->bits, bitSet->blockCapacity * sizeof( uint64_t ) );
	bitSet->blockCapacity = 0;
	bitSet->blockCount = 0;
	bitSet->bits = NULL;
}

void b3SetBitCountAndClear( b3BitSet* bitSet, uint32_t bitCount )
{
	uint32_t blockCount = ( bitCount + sizeof( uint64_t ) * 8 - 1 ) / ( sizeof( uint64_t ) * 8 );
	if ( bitSet->blockCapacity < blockCount )
	{
		b3DestroyBitSet( bitSet );
		uint32_t newBitCapacity = bitCount + ( bitCount >> 1 );
		*bitSet = b3CreateBitSet( newBitCapacity );
	}

	bitSet->blockCount = blockCount;
#if defined( BOX3D_PS2 )
	b3Ps2ClearBytes( bitSet->bits, bitSet->blockCount * sizeof( uint64_t ) );
#else
	memset( bitSet->bits, 0, bitSet->blockCount * sizeof( uint64_t ) );
#endif
}

void b3GrowBitSet( b3BitSet* bitSet, uint32_t blockCount )
{
	B3_ASSERT( blockCount > bitSet->blockCount );
	if ( blockCount > bitSet->blockCapacity )
	{
		uint32_t oldCapacity = bitSet->blockCapacity;
		bitSet->blockCapacity = blockCount + blockCount / 2;
		uint64_t* newBits = (uint64_t*)b3Alloc( bitSet->blockCapacity * sizeof( uint64_t ) );
#if defined( BOX3D_PS2 )
		b3Ps2ClearBytes( newBits, bitSet->blockCapacity * sizeof( uint64_t ) );
#else
		memset( newBits, 0, bitSet->blockCapacity * sizeof( uint64_t ) );
#endif
		B3_ASSERT( bitSet->bits != NULL );
		memcpy( newBits, bitSet->bits, oldCapacity * sizeof( uint64_t ) );
		b3Free( bitSet->bits, oldCapacity * sizeof( uint64_t ) );
		bitSet->bits = newBits;
	}

	bitSet->blockCount = blockCount;
}

void b3InPlaceUnion( b3BitSet* __restrict setA, const b3BitSet* __restrict setB )
{
	B3_ASSERT( setA->blockCount == setB->blockCount );
	uint32_t blockCount = setA->blockCount;
#if defined( BOX3D_PS2 )
	b3Ps2OrWords( setA->bits, setB->bits, blockCount );
#else
	for ( uint32_t i = 0; i < blockCount; ++i )
	{
		setA->bits[i] |= setB->bits[i];
	}
#endif
}
