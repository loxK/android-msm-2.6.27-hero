/*
 * Copyright (c) 2008, NEC Software Tohoku, Ltd.
 * Written by Takashi Sato <t-sato@yk.jp.nec.com>
 *            Akira Fujita <a-fujita@rs.jp.nec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2.1 of the GNU Lesser General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Online defragmentation for EXT4 */

#include <linux/quotaops.h>
#include "ext4_jbd2.h"
#include "ext4_extents.h"
#include "group.h"

/**
 * ext4_defrag_next_extent - Search for the next extent and set it to "extent"
 *
 * @inode:	inode which is searched
 * @path:	this will obtain data for the next extent
 * @extent:	pointer to the next extent we have just gotten
 *
 * This function returns 0 or 1(last entry) if succeed, otherwise
 * returns -EIO.
 */
static int
ext4_defrag_next_extent(struct inode *inode, struct ext4_ext_path *path,
			struct ext4_extent **extent)
{
	int ppos, leaf_ppos = path->p_depth;

	ppos = leaf_ppos;
	if (EXT_LAST_EXTENT(path[ppos].p_hdr) > path[ppos].p_ext) {
		/* leaf block */
		*extent = ++path[ppos].p_ext;
		return 0;
	}

	while (--ppos >= 0) {
		if (EXT_LAST_INDEX(path[ppos].p_hdr) >
		    path[ppos].p_idx) {
			int cur_ppos = ppos;

			/* index block */
			path[ppos].p_idx++;
			path[ppos].p_block = idx_pblock(path[ppos].p_idx);
			if (path[ppos+1].p_bh)
				brelse(path[ppos+1].p_bh);
			path[ppos+1].p_bh =
				sb_bread(inode->i_sb, path[ppos].p_block);
			if (!path[ppos+1].p_bh)
				goto err;
			path[ppos+1].p_hdr =
				ext_block_hdr(path[ppos+1].p_bh);

			/* Halfway index block */
			while (++cur_ppos < leaf_ppos) {
				path[cur_ppos].p_idx =
					EXT_FIRST_INDEX(path[cur_ppos].p_hdr);
				path[cur_ppos].p_block =
					idx_pblock(path[cur_ppos].p_idx);
				if (path[cur_ppos+1].p_bh)
					brelse(path[cur_ppos+1].p_bh);
				path[cur_ppos+1].p_bh = sb_bread(inode->i_sb,
					path[cur_ppos].p_block);
				if (!path[cur_ppos+1].p_bh)
					goto err;
				path[cur_ppos+1].p_hdr =
					ext_block_hdr(path[cur_ppos+1].p_bh);
			}

			/* leaf block */
			path[leaf_ppos].p_ext = *extent =
				EXT_FIRST_EXTENT(path[leaf_ppos].p_hdr);
			return 0;
		}
	}
	/* We found the last extent */
	return 1;
err:
	if (path)
		ext4_ext_drop_refs(path);
	return -EIO;
}

int ext4_defrag_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int err = 0;

	if (!(EXT4_I(inode)->i_flags & EXT4_EXTENTS_FL ||
					cmd == EXT4_IOC_FIBMAP)) {
		printk(KERN_ERR "ext4 defrag: ino[%lu] is not extents "
					"based file\n", inode->i_ino);
		return -EOPNOTSUPP;
	}

	if (cmd == EXT4_IOC_FIBMAP) {
		ext4_fsblk_t __user *p = (ext4_fsblk_t __user *)arg;
		ext4_fsblk_t block = 0;
		struct address_space *mapping = filp->f_mapping;

		if (copy_from_user(&block, (ext4_fsblk_t __user *)arg,
					sizeof(block)))
			return -EFAULT;

		block = ext4_bmap(mapping, block);

		return put_user(block, p);
	} else if (cmd == EXT4_IOC_GROUP_INFO) {
		struct ext4_group_data_info grp_data;

		grp_data.s_blocks_per_group =
			EXT4_BLOCKS_PER_GROUP(inode->i_sb);
		grp_data.s_inodes_per_group =
			EXT4_INODES_PER_GROUP(inode->i_sb);

		if (copy_to_user((struct ext4_group_data_info __user *)arg,
			&grp_data, sizeof(grp_data)))
			return -EFAULT;
	} else if (cmd == EXT4_IOC_DEFRAG) {
		struct ext4_ext_defrag_data defrag;
		struct ext4_super_block *es = EXT4_SB(inode->i_sb)->s_es;

		if (!capable(CAP_DAC_OVERRIDE)) {
			if ((inode->i_mode & S_IRUSR) != S_IRUSR)
				return -EACCES;
			if (current->fsuid != inode->i_uid)
				return -EACCES;
		}

		if (copy_from_user(&defrag,
			(struct ext4_ext_defrag_data __user *)arg,
						sizeof(defrag)))
			return -EFAULT;

		/* Check goal offset if goal offset was given from userspace */
		if (defrag.goal != -1 &&
				ext4_blocks_count(es) <= defrag.goal) {
			printk(KERN_ERR "ext4 defrag: Invalid goal offset"
				" %llu, you can set goal offset up to %llu\n",
				defrag.goal, ext4_blocks_count(es) - 1);
			return -EINVAL;
		}

		err = ext4_defrag(filp, defrag.start_offset,
				defrag.defrag_size, defrag.goal);
	}

	return err;
}

/**
 * ext4_defrag_merge_across_blocks - Merge extents across leaf block
 *
 * @handle:		journal handle
 * @org_inode:		original inode
 * @o_start:		first original extent to be defraged
 * @o_end:		last original extent to be defraged
 * @start_ext:		first new extent to be merged
 * @new_ext:		middle of new extent to be merged
 * @end_ext:		last new extent to be merged
 *
 * This function returns 0 if succeed, otherwise returns error value.
 */
static int
ext4_defrag_merge_across_blocks(handle_t *handle, struct inode *org_inode,
		struct ext4_extent *o_start, struct ext4_extent *o_end,
		struct ext4_extent *start_ext, struct ext4_extent *new_ext,
		struct ext4_extent *end_ext)
{
	struct ext4_ext_path *org_path = NULL;
	ext4_lblk_t eblock = 0;
	int new_flag = 0;
	int end_flag = 0;
	int err;

	if (le16_to_cpu(start_ext->ee_len) &&
		le16_to_cpu(new_ext->ee_len) &&
		le16_to_cpu(end_ext->ee_len)) {

		if (o_start == o_end) {

			/*       start_ext   new_ext    end_ext
			 * dest |---------|-----------|--------|
			 * org  |------------------------------|
			 */

			end_flag = 1;
		} else {

			/*       start_ext   new_ext   end_ext
			 * dest |---------|----------|---------|
			 * org  |---------------|--------------|
			 */

			o_end->ee_block = end_ext->ee_block;
			o_end->ee_len = end_ext->ee_len;
			ext4_ext_store_pblock(o_end, ext_pblock(end_ext));
		}

		o_start->ee_len = start_ext->ee_len;
		new_flag = 1;

	} else if (le16_to_cpu(start_ext->ee_len) &&
			le16_to_cpu(new_ext->ee_len) &&
			!le16_to_cpu(end_ext->ee_len) &&
			o_start == o_end) {

		/*	 start_ext	new_ext
		 * dest |--------------|---------------|
		 * org  |------------------------------|
		 */

		o_start->ee_len = start_ext->ee_len;
		new_flag = 1;

	} else if (!le16_to_cpu(start_ext->ee_len) &&
			le16_to_cpu(new_ext->ee_len) &&
			le16_to_cpu(end_ext->ee_len) &&
			o_start == o_end) {

		/*	  new_ext	end_ext
		 * dest |--------------|---------------|
		 * org  |------------------------------|
		 */

		o_end->ee_block = end_ext->ee_block;
		o_end->ee_len = end_ext->ee_len;
		ext4_ext_store_pblock(o_end, ext_pblock(end_ext));

		/*
		 * Set 0 to the extent block if new_ext was
		 * the first block.
		 */
		if (!new_ext->ee_block)
			eblock = 0;
		else
			eblock = le32_to_cpu(new_ext->ee_block);

		new_flag = 1;
	} else {
		printk(KERN_ERR "ext4 defrag: Unexpected merge case\n");
		return -EIO;
	}

	if (new_flag) {
		org_path = ext4_ext_find_extent(org_inode, eblock, NULL);
		if (IS_ERR(org_path)) {
			err = PTR_ERR(org_path);
			org_path = NULL;
			goto out;
		}
		err = ext4_ext_insert_extent(handle, org_inode,
					org_path, new_ext);
		if (err)
			goto out;
	}

	if (end_flag) {
		org_path = ext4_ext_find_extent(org_inode,
				le32_to_cpu(end_ext->ee_block) - 1, org_path);
		if (IS_ERR(org_path)) {
			err = PTR_ERR(org_path);
			org_path = NULL;
			goto out;
		}
		err = ext4_ext_insert_extent(handle, org_inode,
					org_path, end_ext);
		if (err)
			goto out;
	}
out:
	if (org_path) {
		ext4_ext_drop_refs(org_path);
		kfree(org_path);
	}

	return err;

}

/**
 * ext4_defrag_merge_inside_block - Merge new extent to the extent block
 *
 * @o_start:		first original extent to be merged
 * @o_end:		last original extent to be merged
 * @start_ext:		first new extent to be merged
 * @new_ext:		middle of new extent to be merged
 * @end_ext:		last new extent to be merged
 * @eh:			extent header of target leaf block
 * @replaced:		the number of blocks which will be replaced with new_ext
 * @range_to_move:	used to decide how to merge
 *
 * This function always returns 0.
 */
static int
ext4_defrag_merge_inside_block(struct ext4_extent *o_start,
		struct ext4_extent *o_end, struct ext4_extent *start_ext,
		struct ext4_extent *new_ext, struct ext4_extent *end_ext,
		struct ext4_extent_header *eh, ext4_fsblk_t replaced,
		int range_to_move)
{
	int i = 0;
	unsigned len;

	/* Move the existing extents */
	if (range_to_move && o_end < EXT_LAST_EXTENT(eh)) {
		len = (unsigned long)(EXT_LAST_EXTENT(eh) + 1) -
			(unsigned long)(o_end + 1);
		memmove(o_end + 1 + range_to_move, o_end + 1, len);
	}

	/* Insert start entry */
	if (le16_to_cpu(start_ext->ee_len))
		o_start[i++].ee_len = start_ext->ee_len;

	/* Insert new entry */
	if (le16_to_cpu(new_ext->ee_len)) {
		o_start[i].ee_block = new_ext->ee_block;
		o_start[i].ee_len = cpu_to_le16(replaced);
		ext4_ext_store_pblock(&o_start[i++], ext_pblock(new_ext));
	}

	/* Insert end entry */
	if (end_ext->ee_len)
		o_start[i] = *end_ext;

	/* Increment the total entries counter on the extent block */
	le16_add_cpu(&eh->eh_entries, range_to_move);

	return 0;
}

/**
 * ext4_defrag_merge_extents - Merge new extent
 *
 * @handle:	journal handle
 * @org_inode:	original inode
 * @org_path:	path indicates first extent to be defraged
 * @o_start:	first original extent to be defraged
 * @o_end:	last original extent to be defraged
 * @start_ext:	first new extent to be merged
 * @new_ext:	middle of new extent to be merged
 * @end_ext:	last new extent to be merged
 * @replaced:	the number of blocks which will be replaced with new_ext
 *
 * This function returns 0 if succeed, otherwise returns error value.
 */
static int
ext4_defrag_merge_extents(handle_t *handle, struct inode *org_inode,
		struct ext4_ext_path *org_path,
		struct ext4_extent *o_start, struct ext4_extent *o_end,
		struct ext4_extent *start_ext, struct ext4_extent *new_ext,
		struct ext4_extent *end_ext, ext4_fsblk_t replaced)
{
	struct  ext4_extent_header *eh;
	unsigned need_slots, slots_range;
	int	range_to_move, depth, ret;

	/*
	 * The extents need to be inserted
	 * start_extent + new_extent + end_extent.
	 */
	need_slots = (le16_to_cpu(start_ext->ee_len) ? 1 : 0) +
			(le16_to_cpu(end_ext->ee_len) ? 1 : 0) +
			(le16_to_cpu(new_ext->ee_len) ? 1 : 0);

	/* The number of slots between start and end */
	slots_range = ((unsigned long)(o_end + 1) - (unsigned long)o_start + 1)
					/ sizeof(struct ext4_extent);

	/* Range to move the end of extent */
	range_to_move = need_slots - slots_range;
	depth = org_path->p_depth;
	org_path += depth;
	eh = org_path->p_hdr;

	if (depth) {
		/* Register to journal */
		ret = ext4_journal_get_write_access(handle, org_path->p_bh);
		if (ret)
			return ret;
	}

	/* Expansion */
	if (range_to_move > 0 &&
		(range_to_move > le16_to_cpu(eh->eh_max)
			- le16_to_cpu(eh->eh_entries))) {

		ret = ext4_defrag_merge_across_blocks(handle, org_inode,
					o_start, o_end, start_ext, new_ext,
					end_ext);
		if (ret < 0)
			return ret;
	} else {
		ret = ext4_defrag_merge_inside_block(o_start, o_end,
					start_ext, new_ext, end_ext, eh,
					replaced, range_to_move);
		if (ret < 0)
			return ret;
	}

	if (depth) {
		ret = ext4_journal_dirty_metadata(handle, org_path->p_bh);
		if (ret)
			return ret;
	} else {
		ret = ext4_mark_inode_dirty(handle, org_inode);
		if (ret < 0)
			return ret;
	}

	return 0;

}

/**
 * ext4_defrag_leaf_block - Defragmentation for one leaf extent block
 *
 * @handle:		journal handle
 * @org_inode:		original inode
 * @org_path:		path indicates first extent to be defraged
 * @dext:		destination extent
 * @from:		start offset on the target file
 *
 * This function returns 0 if succeed, otherwise returns error value.
 */
static int
ext4_defrag_leaf_block(handle_t *handle, struct inode *org_inode,
		struct ext4_ext_path *org_path, struct ext4_extent *dext,
		ext4_lblk_t *from)
{
	struct ext4_extent *oext, *o_start = NULL, *o_end = NULL, *prev_ext;
	struct ext4_extent new_ext, start_ext, end_ext;
	ext4_fsblk_t replaced = 0;
	ext4_lblk_t new_end, lblock;
	unsigned long depth;
	unsigned short len;
	ext4_fsblk_t new_phys_end;
	int	ret;

	depth = ext_depth(org_inode);
	start_ext.ee_len = end_ext.ee_len = 0;
	o_start = o_end = oext = org_path[depth].p_ext;
	ext4_ext_store_pblock(&new_ext, ext_pblock(dext));
	new_ext.ee_len = dext->ee_len;
	len = le16_to_cpu(new_ext.ee_len);
	new_ext.ee_block = cpu_to_le32(*from);
	lblock = le32_to_cpu(oext->ee_block);
	new_end = le32_to_cpu(new_ext.ee_block)
		+ le16_to_cpu(new_ext.ee_len) - 1;
	new_phys_end = ext_pblock(&new_ext)
		+ le16_to_cpu(new_ext.ee_len) - 1;

	/*
	 * First original extent
	 * dest	 |---------------|
	 * org  |---------------|
	 */
	if (le32_to_cpu(new_ext.ee_block) >
		le32_to_cpu(oext->ee_block) &&
		le32_to_cpu(new_ext.ee_block) <
		le32_to_cpu(oext->ee_block)
		+ le16_to_cpu(oext->ee_len)) {
		start_ext.ee_len = cpu_to_le16(le32_to_cpu(new_ext.ee_block)
					- le32_to_cpu(oext->ee_block));
		replaced += le16_to_cpu(oext->ee_len)
					- le16_to_cpu(start_ext.ee_len);
	} else if (oext > EXT_FIRST_EXTENT(org_path[depth].p_hdr)) {
		/* We can merge previous extent. */
		prev_ext = oext - 1;
		if (((ext_pblock(prev_ext) + le16_to_cpu(prev_ext->ee_len))
				 == ext_pblock(&new_ext))
		 && (le32_to_cpu(prev_ext->ee_block)
			+ le16_to_cpu(prev_ext->ee_len)
				 == le32_to_cpu(new_ext.ee_block))) {
			o_start = prev_ext;
			start_ext.ee_len = cpu_to_le16(
					le16_to_cpu(prev_ext->ee_len)
					+ le16_to_cpu(new_ext.ee_len));
			new_ext.ee_len = 0;
		}
	}

	for (;;) {
		/* The extent for destination must be found. */
		BUG_ON(!oext || lblock != le32_to_cpu(oext->ee_block));
		lblock += le16_to_cpu(oext->ee_len);

		/*
		 * Middle of original extent
		 * dest |-------------------|
		 * org   |-----------------|
		 */
		if (le32_to_cpu(new_ext.ee_block) <=
			le32_to_cpu(oext->ee_block) &&
			new_end >= le32_to_cpu(oext->ee_block)
			+ le16_to_cpu(oext->ee_len) - 1)
			replaced += le16_to_cpu(oext->ee_len);

		/*
		 * Last original extent
		 * dest |----------------|
		 * org	  |---------------|
		 */
		if (new_end >= le32_to_cpu(oext->ee_block) &&
			new_end < le32_to_cpu(oext->ee_block)
				+ le16_to_cpu(oext->ee_len) - 1) {
			end_ext.ee_len
				= cpu_to_le16(le32_to_cpu(oext->ee_block)
				+ le16_to_cpu(oext->ee_len) - 1 - new_end);
			ext4_ext_store_pblock(&end_ext, (ext_pblock(o_end)
				+ le16_to_cpu(oext->ee_len)
				- le16_to_cpu(end_ext.ee_len)));
			end_ext.ee_block
				= cpu_to_le32(le32_to_cpu(o_end->ee_block)
				+ le16_to_cpu(oext->ee_len)
				- le16_to_cpu(end_ext.ee_len));
			replaced += le16_to_cpu(oext->ee_len)
				- le16_to_cpu(end_ext.ee_len);
		}

		/*
		 * Detected the block end, reached the number of replaced
		 * blocks to dext->ee_len. Then merge the extent.
		 */
		if (oext == EXT_LAST_EXTENT(org_path[depth].p_hdr) ||
			new_end <= le32_to_cpu(oext->ee_block)
				+ le16_to_cpu(oext->ee_len) - 1) {
			ret = ext4_defrag_merge_extents(handle, org_inode,
					org_path, o_start, o_end, &start_ext,
					&new_ext, &end_ext, replaced);
			if (ret < 0)
				return ret;

			/* All expected blocks are replaced */
			if (le16_to_cpu(new_ext.ee_len) <= 0)
				return 0;

			/* Re-calculate new_ext */
			le16_add_cpu(&new_ext.ee_len, -replaced);
			le32_add_cpu(&new_ext.ee_block, replaced);
			ext4_ext_store_pblock(&new_ext, ext_pblock(&new_ext)
					+ replaced);
			replaced = 0;
			start_ext.ee_len = end_ext.ee_len = 0;
			o_start = NULL;

			/* All expected blocks are replaced. */
			if (le16_to_cpu(new_ext.ee_len) <= 0)
				return 0;
		}

		/* Get the next extent for original. */
		if (org_path)
			ext4_ext_drop_refs(org_path);
		org_path = ext4_ext_find_extent(org_inode, lblock, org_path);
		if (IS_ERR(org_path)) {
			ret = PTR_ERR(org_path);
			org_path = NULL;
			return ret;
		}
		depth = ext_depth(org_inode);
		oext = org_path[depth].p_ext;
		if (le32_to_cpu(oext->ee_block) + le16_to_cpu(oext->ee_len)
			<= lblock)
			return -ENOENT;

		o_end = oext;
		if (!o_start)
			o_start = oext;
	}
}

/**
 * ext4_defrag_replace_branches - Replace original extents with new extents
 *
 * @handle:		journal handle
 * @org_inode:		original inode
 * @dest_inode:		temporary inode
 * @from:		block offset of org_inode
 * @dest_off:		block offset of dest_inode
 * @count:		block count to be replaced
 *
 * This function returns 0 if succeed, otherwise returns error value.
 * Replace extents for blocks from "from" to "from + count - 1".
 */
static int
ext4_defrag_replace_branches(handle_t *handle, struct inode *org_inode,
			struct inode *dest_inode, ext4_lblk_t from,
			ext4_lblk_t dest_off, ext4_lblk_t count)
{
	struct ext4_ext_path *org_path = NULL;
	struct ext4_ext_path *dest_path = NULL;
	struct ext4_extent *oext, *dext, *swap_ext;
	struct ext4_extent tmp_ext, tmp_ext2;
	ext4_lblk_t diff, org_diff;
	int err = 0;
	int depth;
	int replaced_count = 0;

	/* Get the original extent for the block "from" */
	org_path = ext4_ext_find_extent(org_inode, from, NULL);
	if (IS_ERR(org_path)) {
		err = PTR_ERR(org_path);
		org_path = NULL;
		goto out;
	}

	/* Get the destination extent for the head */
	dest_path = ext4_ext_find_extent(dest_inode, dest_off, NULL);
	if (IS_ERR(dest_path)) {
		err = PTR_ERR(dest_path);
		dest_path = NULL;
		goto out;
	}
	depth = ext_depth(dest_inode);
	dext = dest_path[depth].p_ext;
	/* When dext is too large, pick up the target range. */
	diff = dest_off - le32_to_cpu(dext->ee_block);
	ext4_ext_store_pblock(&tmp_ext, ext_pblock(dext) + diff);
	tmp_ext.ee_block = cpu_to_le32(le32_to_cpu(dext->ee_block) + diff);
	tmp_ext.ee_len = cpu_to_le16(le16_to_cpu(dext->ee_len) - diff);
	if (count < le16_to_cpu(tmp_ext.ee_len))
		tmp_ext.ee_len = cpu_to_le16(count);
	dext = &tmp_ext;

	depth = ext_depth(org_inode);
	oext = org_path[depth].p_ext;
	org_diff = from - le32_to_cpu(oext->ee_block);
	ext4_ext_store_pblock(&tmp_ext2, ext_pblock(oext) + org_diff);
	tmp_ext2.ee_block = tmp_ext.ee_block;

	/* Adjust extent length when blocksize != pagesize */
	if (le16_to_cpu(tmp_ext.ee_len) <=
		le16_to_cpu(oext->ee_len) - org_diff) {
		tmp_ext2.ee_len = tmp_ext.ee_len;
	} else {
		tmp_ext2.ee_len = cpu_to_le16(le16_to_cpu(oext->ee_len)
						- org_diff);
		tmp_ext.ee_len = tmp_ext2.ee_len;
	}
	swap_ext = &tmp_ext2;

	/* Loop for the destination extents */
	while (1) {
		/* The extent for destination must be found. */
		BUG_ON(!dext || dest_off != le32_to_cpu(dext->ee_block));

		/* Loop for the original extent blocks */
		err = ext4_defrag_leaf_block(handle, org_inode,
						org_path, dext, &from);
		if (err < 0)
			goto out;

		/*
		 * We need the function which fixes extent information for
		 * inserting.
		 * e.g. ext4_defrag_merge_extents()
		 */
		err = ext4_defrag_leaf_block(handle, dest_inode,
					dest_path, swap_ext, &dest_off);
		if (err < 0)
			goto out;

		replaced_count += le16_to_cpu(dext->ee_len);
		dest_off += le16_to_cpu(dext->ee_len);
		from += le16_to_cpu(dext->ee_len);

		/* Already moved the expected blocks */
		if (replaced_count >= count)
			break;

		if (org_path)
			ext4_ext_drop_refs(org_path);
		org_path = ext4_ext_find_extent(org_inode, from, NULL);
		if (IS_ERR(org_path)) {
			err = PTR_ERR(org_path);
			org_path = NULL;
			goto out;
		}
		depth = ext_depth(org_inode);
		oext = org_path[depth].p_ext;
		if (le32_to_cpu(oext->ee_block) + le16_to_cpu(oext->ee_len)
			<= from) {
			err = 0;
			goto out;
		}

		if (dest_path)
			ext4_ext_drop_refs(dest_path);
		dest_path = ext4_ext_find_extent(dest_inode, dest_off, NULL);
		if (IS_ERR(dest_path)) {
			err = PTR_ERR(dest_path);
			dest_path = NULL;
			goto out;
		}
		depth = ext_depth(dest_inode);
		dext = dest_path[depth].p_ext;
		if (le32_to_cpu(dext->ee_block) + le16_to_cpu(dext->ee_len)
			<= dest_off) {
			err = 0;
			goto out;
		}

		/* When dext is too large, pick up the target range. */
		diff = dest_off - le32_to_cpu(dext->ee_block);
		ext4_ext_store_pblock(&tmp_ext, ext_pblock(dext) + diff);
		tmp_ext.ee_block =
			cpu_to_le32(le32_to_cpu(dext->ee_block) + diff);
		tmp_ext.ee_len = cpu_to_le16(le16_to_cpu(dext->ee_len) - diff);

		if (count - replaced_count < le16_to_cpu(tmp_ext.ee_len))
			tmp_ext.ee_len = cpu_to_le16(count - replaced_count);

		dext = &tmp_ext;

		org_diff = from - le32_to_cpu(oext->ee_block);
		ext4_ext_store_pblock(&tmp_ext2, ext_pblock(oext) + org_diff);
		tmp_ext2.ee_block = tmp_ext.ee_block;

		/* Adjust extent length when blocksize != pagesize */
		if (le16_to_cpu(tmp_ext.ee_len) <=
			le16_to_cpu(oext->ee_len) - org_diff) {
			tmp_ext2.ee_len = tmp_ext.ee_len;
		} else {
			tmp_ext2.ee_len = cpu_to_le16(le16_to_cpu(oext->ee_len)
							- org_diff);
			tmp_ext.ee_len = tmp_ext2.ee_len;
		}
		swap_ext = &tmp_ext2;
	}

out:
	if (org_path) {
		ext4_ext_drop_refs(org_path);
		kfree(org_path);
	}
	if (dest_path) {
		ext4_ext_drop_refs(dest_path);
		kfree(dest_path);
	}

	return err;
}

/**
 * ext4_defrag_fill_ar - Prepare to multiple block allocate for tmp inode
 *
 * @org_inode:		original inode
 * @dest_inode:		temporary inode
 * @ar:			allocation request for multiple block allocation
 * @org_path:		indicating the original inode's extent
 * @dest_path:		indicating the temporary inode's extent
 * @req_blocks:		contiguous blocks count we need
 * @iblock:		target file offset
 * @goal:		goal offset
 *
 */
static void
ext4_defrag_fill_ar(struct inode *org_inode, struct inode *dest_inode,
			struct ext4_allocation_request *ar,
			struct ext4_ext_path *org_path,
			struct ext4_ext_path *dest_path,
			ext4_fsblk_t req_blocks, ext4_lblk_t iblock,
			ext4_fsblk_t goal)
{
	ar->inode = dest_inode;
	ar->len = req_blocks;
	ar->logical = iblock;
	ar->flags = EXT4_MB_HINT_DATA | EXT4_MB_HINT_RESERVED
		| EXT4_MB_HINT_NOPREALLOC;
	ar->lleft = 0;
	ar->pleft = 0;
	ar->lright = 0;
	ar->pright = 0;

	if (goal)
		ar->goal = goal;
	else
		ar->goal = ext4_ext_find_goal(dest_inode, dest_path, iblock);
}

/**
 * ext4_defrag_alloc_blocks - Allocate contiguous blocks to temporary inode
 *
 * @handle:		journal handle
 * @org_inode:		original inode
 * @dest_inode:		temporary inode for multiple block allocation
 * @ar:			allocation request for multiple block allocation
 * @dest_path:		indicating the temporary inode's extent
 * @newblock:		start offset of contiguous blocks
 *
 * This function returns 0 if succeed, otherwise returns error value.
 */
static int
ext4_defrag_alloc_blocks(handle_t *handle, struct inode *org_inode,
		struct inode *dest_inode, struct ext4_allocation_request *ar,
		struct ext4_ext_path *dest_path, ext4_fsblk_t *newblock)
{
	struct super_block *sb = org_inode->i_sb;
	struct buffer_head *bh = NULL;
	int err, i, credits = 0;

	credits = ext4_ext_calc_credits_for_single_extent(dest_inode,
							  ar->len, dest_path);
	err = ext4_ext_journal_restart(handle, credits);
	if (err)
		return err;

	*newblock = ext4_mb_new_blocks(handle, ar, &err);
	if (err)
		return err;

	/*
	 * Dirty buffer_head causes the overwriting
	 * if ext4_mb_new_blocks() allocates the block
	 * which used to be the metadata block.
	 * We should call unmap_underlying_metadata()
	 * to clear the dirty flag.
	 */
	for (i = 0; i < ar->len; i++) {
		bh = sb_find_get_block(sb, *newblock + i);
		unmap_underlying_metadata(sb->s_bdev, *newblock + i);
	}

	return err;
}

/**
 * ext4_defrag_partial - Defrag a file per page
 *
 * @tmp_inode:			temporary inode
 * @filp:			pointer to file
 * @org_page_offset:		page index on original file
 * @dest_blk_offset:		block index on temporary file
 * @data_offset_in_page:	block index where data swapping starts
 * @block_len_in_page:		the number of blocks to be swapped
 *
 * This function returns 0 if succeed, otherwise returns error value.
 */
static int
ext4_defrag_partial(struct inode *tmp_inode, struct file *filp,
			pgoff_t org_page_offset, ext4_lblk_t dest_blk_offset,
			int data_offset_in_page, int block_len_in_page)
{
	struct inode *org_inode = filp->f_dentry->d_inode;
	struct address_space *mapping = org_inode->i_mapping;
	struct buffer_head *bh;
	struct page *page = NULL;
	const struct address_space_operations *a_ops = mapping->a_ops;
	handle_t *handle;
	ext4_lblk_t org_blk_offset;
	long long offs = org_page_offset << PAGE_CACHE_SHIFT;
	unsigned long blocksize = org_inode->i_sb->s_blocksize;
	unsigned int w_flags = 0;
	unsigned int tmp_data_len;
	unsigned data_len;
	void *fsdata;
	int ret, i, jblocks;
	int blocks_per_page = PAGE_CACHE_SIZE >> org_inode->i_blkbits;

	/*
	 * It needs twice the amount of ordinary journal buffers because
	 * inode and tmp_inode may change each different metadata blocks.
	 */
	jblocks = ext4_writepage_trans_blocks(org_inode) * 2;
	handle = ext4_journal_start(org_inode, jblocks);
	if (IS_ERR(handle)) {
		ret = PTR_ERR(handle);
		return ret;
	}

	if (segment_eq(get_fs(), KERNEL_DS))
		w_flags |= AOP_FLAG_UNINTERRUPTIBLE;

	org_blk_offset = org_page_offset * blocks_per_page +
							data_offset_in_page;
	offs = (long long)org_blk_offset << org_inode->i_blkbits;

	/* Calculate data_len */
	if ((org_blk_offset + block_len_in_page - 1) ==
			((org_inode->i_size - 1) >> org_inode->i_blkbits)) {
		/* the case which we replace the last block */
		tmp_data_len = org_inode->i_size & (blocksize - 1);
		/*
		 * If data_len equal zero, it shows data_len is multiples of
		 * blocksize. So we set appropriate value.
		 */
		if (tmp_data_len == 0)
			tmp_data_len = blocksize;

		data_len = tmp_data_len +
			((block_len_in_page - 1) << org_inode->i_blkbits);
	} else {
		data_len = block_len_in_page << org_inode->i_blkbits;
	}

	up_write(&EXT4_I(org_inode)->i_data_sem);
	ret = a_ops->write_begin(filp, mapping, offs, data_len, w_flags, &page,
								&fsdata);
	down_write(&EXT4_I(org_inode)->i_data_sem);

	if (unlikely(ret < 0))
		goto out;

	if (!PageUptodate(page)) {
		up_write(&EXT4_I(org_inode)->i_data_sem);
		mapping->a_ops->readpage(filp, page);
		down_write(&EXT4_I(org_inode)->i_data_sem);
		lock_page(page);
	}

	/*
	 * try_to_release_page() doesn't call releasepage in writeback mode.
	 * We should care about the order of writing to the same file
	 * by multiple defrag processes.
	 * It needs to call wait_on_page_writeback() to wait for the
	 * writeback of the page.
	 */
	if (PageWriteback(page))
		wait_on_page_writeback(page);

	/* Release old bh and drop refs */
	try_to_release_page(page, 0);
	ret = ext4_defrag_replace_branches(handle, org_inode, tmp_inode,
						org_blk_offset, dest_blk_offset,
						block_len_in_page);
	if (ret < 0)
		goto out;

	/* Clear the inode cache not to refer to the old data */
	ext4_ext_invalidate_cache(org_inode);

	if (!page_has_buffers(page))
		create_empty_buffers(page, 1 << org_inode->i_blkbits, 0);

	bh = page_buffers(page);
	for (i = 0; i < data_offset_in_page; i++)
		bh = bh->b_this_page;

	for (i = 0; i < block_len_in_page; i++) {
		up_write(&EXT4_I(org_inode)->i_data_sem);
		ret = ext4_get_block(org_inode, (sector_t)(org_blk_offset + i),
									bh, 0);
		down_write(&EXT4_I(org_inode)->i_data_sem);

		if (ret < 0)
			goto out;

		if (bh->b_this_page != NULL)
			bh = bh->b_this_page;
	}

	ret = a_ops->write_end(filp, mapping, offs, data_len, data_len, page,
									fsdata);
	page = NULL;

out:
	if (unlikely(page)) {
		if (PageLocked(page))
			unlock_page(page);
		page_cache_release(page);
	}
	ext4_journal_stop(handle);

	return ret < 0 ? ret : 0;
}

/**
 * ext4_defrag_comp_ext_count- Check whether fragments are improved or not
 *
 * @org_inode:		original inode
 * @path:		the structure holding some info about
 *			original extent tree
 * @tar_end:		the last block number of the allocated blocks
 * @sum_tmp:		the extents count  in the allocated blocks
 * @goal:		block offset for allocation
 *
 * This function returns the values as below.
 *	0 (improved)
 *	1 (not improved)
 *	negative value (error case)
 */
static int
ext4_defrag_comp_ext_count(struct inode *org_inode,
			struct ext4_ext_path *org_path, ext4_lblk_t tar_end,
			int sum_tmp, ext4_fsblk_t goal)
{
	struct ext4_extent *ext = NULL;
	int depth = ext_depth(org_inode);
	int last_extent = 0;
	int sum_org = 0;
	int ret = 0;

	ext = org_path[depth].p_ext;

	/*
	 * Compare the number of the newly allocated extents to
	 * that of existing one.
	 */
	while (1) {
		if (!last_extent)
			++sum_org;
		if (tar_end <= (le32_to_cpu(ext->ee_block) +
			       le16_to_cpu(ext->ee_len) - 1) ||
			       last_extent) {
			/*
			 * Fail if goal is not set and the fragmentation
			 * is not improved.
			 */
			if (sum_org == sum_tmp && !goal) {
				/* Not improved */
				ret = 1;
			} else if (sum_org < sum_tmp) {
				/* Fragment increased */
				ret = -ENOSPC;
				printk(KERN_ERR "ext4 defrag: "
					"Insufficient free blocks\n");
			}
			break;
		}
		last_extent =
			ext4_defrag_next_extent(org_inode, org_path, &ext);
		if (last_extent < 0) {
			ret = last_extent;
			break;
		}
	}

	return ret;
}

/**
 * ext4_defrag_new_extent_tree - Get contiguous blocks and build an extent tree
 *
 * @org_inode:		original inode
 * @tmp_inode:		temporary inode
 * @org_path:		indicating the original inode's extent
 * @req_start:		starting offset to allocate in blocks
 * @req_blocks:		the number of blocks to allocate
 * @iblock:		file related offset
 * @goal:		block offset for allocation
 *
 * This function returns the value as below:
 *	0 (succeed)
 *	1 (not improved)
 *	negative value (error case)
 */
static int
ext4_defrag_new_extent_tree(struct inode *org_inode, struct inode *tmp_inode,
			struct ext4_ext_path *org_path, ext4_lblk_t req_start,
			ext4_lblk_t req_blocks, ext4_lblk_t iblock,
			ext4_fsblk_t goal)
{
	handle_t *handle;
	struct ext4_sb_info *sbi = EXT4_SB(org_inode->i_sb);
	struct ext4_extent_header *eh = NULL;
	struct ext4_allocation_request ar;
	struct ext4_ext_path *dest_path = NULL;
	struct ext4_extent newex;
	ext4_fsblk_t alloc_total = 0;
	ext4_fsblk_t newblock = 0;
	ext4_lblk_t req_end = req_start + req_blocks - 1;
	ext4_lblk_t rest_blocks = 0;
	int sum_tmp = 0;
	int metadata = 1;
	int ret;

	eh = ext_inode_hdr(tmp_inode);
	eh->eh_depth = 0;

	dest_path = ext4_ext_find_extent(tmp_inode, iblock, NULL);
	if (IS_ERR(dest_path)) {
		ret = PTR_ERR(dest_path);
		dest_path = NULL;
		goto out2;
	}

	/* Fill struct ext4_allocation_request with necessary info */
	ext4_defrag_fill_ar(org_inode, tmp_inode, &ar, org_path,
				dest_path, req_blocks, iblock, goal);

	handle = ext4_journal_start(tmp_inode, 0);
	if (IS_ERR(handle)) {
		ret = PTR_ERR(handle);
		goto out2;
	}

	while (alloc_total != req_blocks) {
		/* Allocate blocks */
		ret = ext4_defrag_alloc_blocks(handle, org_inode, tmp_inode,
						&ar, dest_path, &newblock);
		if (ret < 0)
			goto out;
		/* Claimed blocks are already reserved */
		EXT4_I(ar.inode)->i_delalloc_reserved_flag = 1;

		alloc_total += ar.len;
		rest_blocks = req_blocks - alloc_total;

		newex.ee_block = cpu_to_le32(alloc_total - ar.len);
		ext4_ext_store_pblock(&newex, newblock);
		newex.ee_len = cpu_to_le16(ar.len);

		ret = ext4_ext_insert_extent(handle, tmp_inode,
						dest_path, &newex);
		if (ret < 0)
			goto out;

		ar.goal = newblock + ar.len;
		ar.len = req_blocks - alloc_total;
		sum_tmp++;
	}

	ret = ext4_defrag_comp_ext_count(org_inode, org_path, req_end,
					sum_tmp, goal);

out:
	if (ret < 0 && ar.len)
		ext4_free_blocks(handle, tmp_inode, newblock, ar.len, metadata);
	/*
	 * Update dirty-blocks counter if we cannot allocate the all of
	 * requested blocks.
	 */
	if (rest_blocks)
		percpu_counter_sub(&sbi->s_dirtyblocks_counter, rest_blocks);

	ext4_journal_stop(handle);

out2:
	if (dest_path) {
		ext4_ext_drop_refs(dest_path);
		kfree(dest_path);
	}

	return ret;
}

/**
 * ext4_defrag_check - Check the environment whether a defrag can be done
 *
 * @org_inode:		original inode
 * @defrag_size:	size of defrag in blocks
 * @goal:		pointer to block offset for allocation
 *
 * This function returns 0 if succeed, otherwise returns error value.
 */
static int
ext4_defrag_check(struct inode *org_inode, ext4_lblk_t defrag_size,
		ext4_fsblk_t *goal)
{
	return 0;
}

/**
 * ext4_defrag_init_tmp_inode - Create a temporary inode
 *
 * @org_inode:		original inode
 *
 * This function returns pointer to the struct inode if succeed,
 * otherwise returns error value.
 */
static struct inode *
ext4_defrag_init_tmp_inode(struct inode *org_inode)
{
	handle_t *handle;
	struct inode *tmp_inode;

	handle = ext4_journal_start(org_inode,
		EXT4_DATA_TRANS_BLOCKS(org_inode->i_sb) +
		EXT4_INDEX_EXTRA_TRANS_BLOCKS + 4 +
		2 * EXT4_QUOTA_INIT_BLOCKS(org_inode->i_sb));
	if (IS_ERR(handle))
		/* Return error code */
		return (struct inode *)handle;

	tmp_inode = ext4_new_inode(handle,
		org_inode->i_sb->s_root->d_inode, S_IFREG);
	if (IS_ERR(tmp_inode))
		goto out;

	i_size_write(tmp_inode, i_size_read(org_inode));
	tmp_inode->i_nlink = 0;
	ext4_ext_tree_init(handle, tmp_inode);
	ext4_orphan_add(handle, tmp_inode);

out:
	ext4_journal_stop(handle);

	return tmp_inode;
}

/**
 * ext4_defrag - Defrag the specified range of a file
 *
 * If no-option is specified, ext4_defrag() proceeds the following order.
 * 1.ext4_defrag() calculates the block number where defrag terminates
 *   by the start block number(defrag_start) and the size of defraged data
 *   (defrag_size) specified as arguments.
 *   If the defrag_start points a hole, the extent's start offset pointed by
 *   ext_cur(current extent), holecheck_path, org_path are set after
 *   hole behind.
 * 2.Continue step 3 to step 5, until the holecheck_path points to last_extent
 *   or the ext_cur exceeds the block_end which is last logical block number.
 * 3.To get a length of continues area, call ext4_defrag_next_extent()
 *   specified with the ext_cur(initial value is holecheck_path) re-cursive,
 *   until find un-continuous extent, the start logical block number exceeds
 *   the block_end or the extent points to the last extent.
 * 4.After determining the length of continuous block,
 *   allocates continuous blocks to a temporary inode
 *   by ext4_defrag_new_extent_tree().
 * 5.Exchange the original inode data with temporary inode data
 *   from org_page_offset to seq_end_page.
 *   The start indexes of data are specified as arguments.
 *   That of the original inode is org_page_offset,
 *   and that of the temporary inode is dest_block_offset
 *   (To easily handle blocksize != pagesize case, the offset for the
 *    temporary inode is block unit).
 * 6.Update holecheck_path and org_path to points a next proceeding extent,
 *   and release the temporary inode holding the original fragmented data.
 *   Then, returns to step 2.
 * 7.Release holecheck_path, org_path and temporary inode,
 *    and returns the defrag_size which is the size of defraged data.
 *    The defrag_size is used for the command to calculate the file offset
 *    where a next defrag processing start.
 *    (Since the defrag command calls defrag_ioctl() by 64MB unit,
 *     a file bigger than 64MB calls defrag_ioctl many times.)
 *
 * @filp:		pointer to file
 * @block_start:	starting offset to defrag in blocks
 * @defrag_size:	size of defrag in blocks
 * @goal:		block offset for allocation
 *
 * This function returns the number of blocks if succeed, otherwise
 * returns error value.
 */
int
ext4_defrag(struct file *filp, ext4_lblk_t block_start,
		ext4_lblk_t defrag_size, ext4_fsblk_t goal)
{
	struct inode *org_inode = filp->f_dentry->d_inode, *tmp_inode = NULL;
	struct ext4_ext_path *org_path = NULL, *holecheck_path = NULL;
	struct ext4_extent *ext_prev, *ext_cur, *ext_dummy;
	ext4_lblk_t block_end, seq_start, add_blocks, file_end, seq_blocks = 0;
	ext4_lblk_t dest_block_offset;
	ext4_lblk_t rest_blocks;
	pgoff_t org_page_offset, seq_end_page;
	int ret, depth, seq_extents, last_extent = 0;
	int blocks_per_page;
	int data_offset_in_page;
	int block_len_in_page;

	/* Check the filesystem environment whether defrag can be done */
	ret = ext4_defrag_check(org_inode, defrag_size, &goal);
	if (ret < 0)
		return ret;

	blocks_per_page = PAGE_CACHE_SIZE >> org_inode->i_blkbits;
	file_end = (org_inode->i_size - 1) >> org_inode->i_blkbits;
	block_end = block_start + defrag_size - 1;
	if (file_end < block_end)
		defrag_size -= block_end - file_end;

	mutex_lock(&org_inode->i_mutex);
	down_write(&EXT4_I(org_inode)->i_data_sem);

	org_path = ext4_ext_find_extent(org_inode, block_start, NULL);
	if (IS_ERR(org_path)) {
		ret = PTR_ERR(org_path);
		org_path = NULL;
		goto out;
	}

	/* Get path structure to check the hole */
	holecheck_path = ext4_ext_find_extent(org_inode, block_start, NULL);
	if (IS_ERR(holecheck_path)) {
		ret = PTR_ERR(holecheck_path);
		holecheck_path = NULL;
		goto out;
	}

	depth = ext_depth(org_inode);
	ext_cur = holecheck_path[depth].p_ext;
	if (ext_cur == NULL)
		goto out;

	/*
	 * Get proper extent whose ee_block is beyond block_start
	 * if block_start was within the hole.
	 */
	if (le32_to_cpu(ext_cur->ee_block) +
		le16_to_cpu(ext_cur->ee_len) - 1 < block_start) {
		last_extent = ext4_defrag_next_extent(org_inode,
					holecheck_path, &ext_cur);
		if (last_extent < 0) {
			ret = last_extent;
			goto out;
		}
		last_extent = ext4_defrag_next_extent(org_inode, org_path,
							&ext_dummy);
		if (last_extent < 0) {
			ret = last_extent;
			goto out;
		}
	}
	seq_extents = 1;
	seq_start = le32_to_cpu(ext_cur->ee_block);

	/* No blocks within the specified range. */
	if (le32_to_cpu(ext_cur->ee_block) > block_end) {
		printk(KERN_INFO "ext4 defrag: The specified range of file"
				" may be the hole\n");
		goto out;
	}

	/* Adjust start blocks */
	add_blocks = min(le32_to_cpu(ext_cur->ee_block) +
			 le16_to_cpu(ext_cur->ee_len), block_end + 1) -
		     max(le32_to_cpu(ext_cur->ee_block), block_start);

	while (!last_extent && le32_to_cpu(ext_cur->ee_block) <= block_end) {
		seq_blocks += add_blocks;

		/* Create a temporary inode to be exchanged data block */
		tmp_inode = ext4_defrag_init_tmp_inode(org_inode);
		if (IS_ERR(tmp_inode)) {
			ret = PTR_ERR(tmp_inode);
			tmp_inode = NULL;
			goto out;
		}

		/* Adjust tail blocks */
		if (seq_start + seq_blocks - 1 > block_end)
			seq_blocks = block_end - seq_start + 1;

		ext_prev = ext_cur;
		last_extent = ext4_defrag_next_extent(org_inode,
					holecheck_path, &ext_cur);
		if (last_extent < 0) {
			ret = last_extent;
			break;
		}
		if (!last_extent)
			seq_extents++;
		add_blocks = le16_to_cpu(ext_cur->ee_len);

		/*
		 * Extend the length of contiguous block (seq_blocks)
		 * if extents are contiguous.
		 */
		if (le32_to_cpu(ext_prev->ee_block) +
				le16_to_cpu(ext_prev->ee_len) ==
				le32_to_cpu(ext_cur->ee_block) &&
				block_end >= le32_to_cpu(ext_cur->ee_block) &&
				!last_extent) {
			if (tmp_inode) {
				iput(tmp_inode);
				tmp_inode = NULL;
			}
			continue;
		}

		/* Found an isolated block */
		if (seq_extents == 1 && !goal) {
			seq_start = le32_to_cpu(ext_cur->ee_block);
			goto CLEANUP;
		}

		ret = ext4_defrag_new_extent_tree(org_inode, tmp_inode,
					org_path, seq_start, seq_blocks,
					block_start, goal);

		if (ret < 0) {
			break;
		} else if (ret == 1) {
			ret = 0;
			seq_start = le32_to_cpu(ext_cur->ee_block);
			goto CLEANUP;
		}

		data_offset_in_page = seq_start % blocks_per_page;

		/*
		 * Calculate data blocks count that should be swapped
		 * at the first page.
		 */
		if (data_offset_in_page + seq_blocks > blocks_per_page) {
			/* Swapped blocks are across pages */
			block_len_in_page =
					blocks_per_page - data_offset_in_page;
		} else {
			/* Swapped blocks are in a page */
			block_len_in_page = seq_blocks;
		}

		org_page_offset = seq_start >>
				(PAGE_CACHE_SHIFT - org_inode->i_blkbits);
		seq_end_page = (seq_start + seq_blocks - 1) >>
				(PAGE_CACHE_SHIFT - org_inode->i_blkbits);
		seq_start = le32_to_cpu(ext_cur->ee_block);
		rest_blocks = seq_blocks;
		/* Offset for the tmp_inode */
		dest_block_offset = 0;

		/*
		 * Discard all preallocations.
		 * This is provisional solution.
		 * When true ext4_mb_return_to_preallocation() is
		 * implemented, this will be removed.
		 */
		ext4_discard_preallocations(org_inode);

		while (org_page_offset <= seq_end_page) {

			/* Swap original branches with new branches */
			ret = ext4_defrag_partial(tmp_inode, filp,
						org_page_offset,
						dest_block_offset,
						data_offset_in_page,
						block_len_in_page);
			if (ret < 0)
				goto out;

			org_page_offset++;
			dest_block_offset += block_len_in_page;

			data_offset_in_page = 0;
			rest_blocks -= block_len_in_page;
			if (rest_blocks > blocks_per_page)
				block_len_in_page = blocks_per_page;
			else
				block_len_in_page = rest_blocks;
		}

		/* Decrease buffer counter */
		if (holecheck_path)
			ext4_ext_drop_refs(holecheck_path);
		holecheck_path = ext4_ext_find_extent(org_inode,
						seq_start, holecheck_path);
		if (IS_ERR(holecheck_path)) {
			ret = PTR_ERR(holecheck_path);
			holecheck_path = NULL;
			break;
		}
		depth = holecheck_path->p_depth;

CLEANUP:
		/* Decrease buffer counter */
		if (org_path)
			ext4_ext_drop_refs(org_path);
		org_path = ext4_ext_find_extent(org_inode, seq_start, org_path);
		if (IS_ERR(org_path)) {
			ret = PTR_ERR(org_path);
			org_path = NULL;
			break;
		}

		ext_cur = holecheck_path[depth].p_ext;
		add_blocks = le16_to_cpu(ext_cur->ee_len);
		seq_blocks = 0;
		seq_extents = 1;

		if (tmp_inode) {
			iput(tmp_inode);
			tmp_inode = NULL;
		}
	}

out:
	if (org_path) {
		ext4_ext_drop_refs(org_path);
		kfree(org_path);
	}
	if (holecheck_path) {
		ext4_ext_drop_refs(holecheck_path);
		kfree(holecheck_path);
	}

	up_write(&EXT4_I(org_inode)->i_data_sem);
	mutex_unlock(&org_inode->i_mutex);

	if (tmp_inode)
		iput(tmp_inode);

	return ret ? ret : defrag_size;
}
