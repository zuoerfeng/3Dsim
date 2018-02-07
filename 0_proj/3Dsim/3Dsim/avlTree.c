#define _CRTDBG_MAP_ALLOC

#include <stdlib.h>
#include <crtdbg.h>
#include "avlTree.h"



/******************************************************************** 
* 
* avlTreeHigh(TREE_NODE *pNode)
* 
* 计算当前树的高度
*  
* Returns         : 树的高度
* 
*********************************************************************/ 
int avlTreeHigh(TREE_NODE *pNode)
{
	int lh=0,rh=0;
	if(!pNode)
		return 0;

	lh = avlTreeHigh(pNode->left_child);
	rh = avlTreeHigh(pNode->right_child);

	return (1+((lh>rh)?lh:rh));
}


/******************************************************************** 
* 
* avlTreeCheck(tAVLTree *pTree , TREE_NODE *pNode)
* 
* 检验当前的有序平衡二叉树是否平衡
* 是否是有序的，并且各节点的指针没有
* 错误
* 
* Returns         : 
* 			  1 : 表示是一棵完备的有序平衡二叉树	
*			  0 : 表示是一棵 不健康的二叉树
*                             不健康可能是不平衡，可能是平衡因子
*                             有错误，也可能是指针不匹配
*********************************************************************/ 
int avlTreeCheck(tAVLTree *pTree , TREE_NODE *pNode)
{
	int lh=0,rh=0;
	TREE_NODE *tree_root = AVL_NULL;

	if(!pTree || !pNode)
		return 0;

	lh = avlTreeHigh(pNode->left_child);
	rh = avlTreeHigh(pNode->right_child);
	if(pNode->bf != lh-rh)   /*平衡因子是正确的*/
		return 0;

	/*存在左子树，但是左子树要大于自己*/
	if(pNode->left_child && ((*pTree->keyCompare)(pNode , pNode->left_child))>=0)
		return 0;

	/*存在右子树，但是右子树要大于自己*/
	if(pNode->right_child && ((*pTree->keyCompare)(pNode , pNode->right_child))<=0)
		return 0;

	/*如果本节点的父亲节点为空，但是树根不是自己*/
	tree_root = pNode->tree_root;
	if(!tree_root && (pTree->pTreeHeader != pNode))
		return 0;

	if(tree_root)
	{
		/******************************
		*父亲节点的左右子树都不是自己或
		*父亲节点的左右子树都是自己
		*******************************/
		if((tree_root->left_child != pNode && tree_root->right_child != pNode) ||
			(tree_root->left_child == pNode && tree_root->right_child == pNode))
			return 0;
	}

	/****************************
	*左子树的父亲节点不是自己或者
	*右子树的父亲节点不是自己
	*****************************/
	if((pNode->left_child && pNode->left_child->tree_root != pNode) ||
		(pNode->right_child && pNode->right_child->tree_root != pNode))
		return 0;

	if(pNode->left_child && !avlTreeCheck(pTree, pNode->left_child))
		return 0;

	if(pNode->right_child && !avlTreeCheck(pTree, pNode->right_child))
		return 0;

	return 1;
}


/******************************************************************** 
* 
* R_Rotate(TREE_NODE **ppNode)
* 
* 二叉树以*ppNode为根节点，进行右旋转操作
* 
* Returns         :  无
*
*           字母后面的数字表示是平衡因子
*
*             E2                C0  
*            / \               / \                    
*           C1  F0            B1  E0                  
*          / \       ==>     /   / \                        
*         B1  D0            A0  D0  F0                      
*        /                                                
*       A0                                                 
*                                              

*                                              
**********************************************************************/
static void R_Rotate(TREE_NODE **ppNode)
{
	TREE_NODE *l_child = AVL_NULL;
	TREE_NODE *pNode = (TREE_NODE *)(*ppNode);

	l_child = pNode->left_child;
	pNode->left_child = l_child->right_child;
	if(l_child->right_child)
		l_child->right_child->tree_root = pNode;
	l_child->right_child = pNode;
	l_child->tree_root = pNode->tree_root;
	pNode->tree_root = l_child;
	(*ppNode) = l_child;
}


/******************************************************************** 
* 
* L_Rotate(TREE_NODE **ppNode)
* 
* 二叉树以*ppNode为根节点，进行左旋转操作
* 
* Returns         :  无
*                   
*          字母后面的数字表示是平衡因子
*                             
*           B-2                  D0                
*          / \       ==>        / \                      
*         A0  D-1              B0  E0                     
*            / \              / \   \                   
*           C0  E-1          A0  C0  F0                  
*                \
*                 F0       
*******************************************************************/ 
static void L_Rotate(TREE_NODE **ppNode)
{
	TREE_NODE *r_child = AVL_NULL;
	TREE_NODE *pNode = (TREE_NODE *)(*ppNode);

	r_child = pNode->right_child;
	pNode->right_child = r_child->left_child;
	if(r_child->left_child)
		r_child->left_child->tree_root = pNode;
	r_child->left_child = pNode;
	r_child->tree_root = pNode->tree_root;
	pNode->tree_root = r_child;
	(*ppNode) = r_child;
}


/******************************************************************** 
* 
* LeftBalance(TREE_NODE **ppNode)
* 
* 二叉树*ppNode左边偏高，失去平衡，进行左平衡操作
* 
* Returns         :  无
********************************************************************/ 
static void LeftBalance(TREE_NODE **ppNode)
{
	TREE_NODE *left_child = AVL_NULL;
	TREE_NODE *right_child = AVL_NULL;
	TREE_NODE *tree_root = AVL_NULL;
	TREE_NODE *pNode = (TREE_NODE *)(*ppNode);

	tree_root = pNode->tree_root;               /*保存当前节点的父节点*/
	left_child = pNode->left_child;             /*保存当前节点的左子树*/
	switch(left_child->bf)
	{
	case LH_FACTOR:                             /*如果左子树的平衡因子为1，证明原始状态为左子树比右子树高*/
		pNode->bf = left_child->bf = EH_FACTOR; /*当前节点的平衡因子和左子树的平衡因子设为0*/
		R_Rotate(ppNode);  /*当前子树右旋*/
		break;
	case RH_FACTOR:                             /*如果左子树的平衡因子为-1，证明原始状态为右子树比左子树高*/
		                                        /*那么平衡因子的计算就需要根据右子树的平衡因子来计算*/
		right_child = left_child->right_child;
		switch(right_child->bf)
		{
		case LH_FACTOR:
			pNode->bf = RH_FACTOR;
			left_child->bf = EH_FACTOR;
			break;
		case EH_FACTOR:
			pNode->bf = left_child->bf = EH_FACTOR;
			break;
		case RH_FACTOR:
			pNode->bf = EH_FACTOR;
			left_child->bf = LH_FACTOR;
			break;
		}
		right_child->bf = EH_FACTOR;
		L_Rotate(&pNode->left_child);          /*将本节点的左子树进行左旋*/
		R_Rotate(ppNode);                      /*将本节点进行右旋*/
		break;
	case EH_FACTOR:                            /*左子树的平衡因子为0，表明原始状态下该子树是平衡的*/
		pNode->bf = LH_FACTOR;
		left_child->bf = RH_FACTOR;
		R_Rotate(ppNode);                     /*将本节点进行右旋*/
		break;
	}
	(*ppNode)->tree_root = tree_root;
	if(tree_root && tree_root->left_child == pNode)
		tree_root->left_child = *ppNode;
	if(tree_root && tree_root->right_child == pNode)
		tree_root->right_child = *ppNode;
}


/******************************************************************** 
* 
* RightBalance(TREE_NODE **ppNode)
* 
* 二叉树*ppNode右边偏高，失去平衡，进行右平衡操作
* 
* Returns         :  无
********************************************************************/ 
static void RightBalance(TREE_NODE **ppNode)
{
	TREE_NODE *left_child = AVL_NULL;
	TREE_NODE *right_child = AVL_NULL;
	TREE_NODE *tree_root = AVL_NULL;
	TREE_NODE *pNode = (TREE_NODE *)(*ppNode);

	tree_root = pNode->tree_root;
	right_child = pNode->right_child;
	switch(right_child->bf)
	{
	case RH_FACTOR:
		pNode->bf = right_child->bf = EH_FACTOR;
		L_Rotate(ppNode);
		break;
	case LH_FACTOR:
		left_child = right_child->left_child;
		switch(left_child->bf)
		{
		case RH_FACTOR:
			pNode->bf = LH_FACTOR;
			right_child->bf = EH_FACTOR;
			break;
		case EH_FACTOR:
			pNode->bf = right_child->bf = EH_FACTOR;
			break;
		case LH_FACTOR:
			pNode->bf = EH_FACTOR;
			right_child->bf = RH_FACTOR;
			break;
		}
		left_child->bf = EH_FACTOR;
		R_Rotate(&pNode->right_child);
		L_Rotate(ppNode);
		break;
	case EH_FACTOR:
		pNode->bf = RH_FACTOR;
		right_child->bf = LH_FACTOR;
		L_Rotate(ppNode);
		break;
	}
	(*ppNode)->tree_root = tree_root;
	if(tree_root && tree_root->left_child == pNode)
		tree_root->left_child = *ppNode;
	if(tree_root && tree_root->right_child == pNode)
		tree_root->right_child = *ppNode;
}


/******************************************************************** 
* 
* avlDelBalance(tAVLTree *pTree , TREE_NODE *pNode,int L_R_MINUS)
* 
* 删除节点之后，二叉树可能已经不平衡了，此时需要用
* 此函数来实现删除节点之后的平衡操作。
* 子树自平衡的过程中，可能出现一种情况：那就是子树自身平衡了，但是
* 破坏了父亲的平衡性，所以此函数做了递归平衡操作，能够使最小不平衡
* 子树之上的所有祖先节点都能够平衡。
* 最坏可能的情况就是从最小不平衡字数的树根一直到整个大树的树根节点
* 之间的所有子树都不平衡，不过这种概率很低，一般来说递归最多三次就
* 可以实现整个树的平衡
* pTree 		:  二叉树指针
* pNode		:  最小不平衡子树的根节点
* L_R_MINUS	:  
*			LEFT_MINUS    -- 左边失去平衡，树高减少了1层
*                      RIGHT_MINUS  -- 右边失去平衡，树高减少了1层
*
* Returns         :  无
******************************************************************/ 
static int avlDelBalance
(
 tAVLTree *pTree , 
 TREE_NODE *pNode,
 int L_R_MINUS
 )
{
	TREE_NODE *tree_root = AVL_NULL;

	tree_root = pNode->tree_root;
	if(L_R_MINUS == LEFT_MINUS)
	{
		switch(pNode->bf)
		{
		case EH_FACTOR:
			pNode->bf = RH_FACTOR;
			break;
		case RH_FACTOR:
			RightBalance(&pNode);
			if(!tree_root)
				pTree->pTreeHeader = pNode;
			if(pNode->tree_root && pNode->bf == EH_FACTOR)
			{
				if(pNode->tree_root->left_child == pNode)
					avlDelBalance(pTree , pNode->tree_root , LEFT_MINUS);
				else
					avlDelBalance(pTree , pNode->tree_root , RIGHT_MINUS);
			}
			break;
		case LH_FACTOR:
			pNode->bf = EH_FACTOR;
			if(pNode->tree_root && pNode->bf == EH_FACTOR)
			{
				if(pNode->tree_root->left_child == pNode)
					avlDelBalance(pTree , pNode->tree_root , LEFT_MINUS);
				else
					avlDelBalance(pTree , pNode->tree_root , RIGHT_MINUS);
			}
			break;
		}
	}

	if(L_R_MINUS == RIGHT_MINUS)
	{
		switch(pNode->bf)
		{
		case EH_FACTOR:
			pNode->bf = LH_FACTOR;
			break;
		case LH_FACTOR:
			LeftBalance(&pNode);
			if(!tree_root)
				pTree->pTreeHeader = pNode;
			if(pNode->tree_root && pNode->bf == EH_FACTOR)
			{
				if(pNode->tree_root->left_child == pNode)
					avlDelBalance(pTree , pNode->tree_root , LEFT_MINUS);
				else
					avlDelBalance(pTree , pNode->tree_root , RIGHT_MINUS);
			}
			break;
		case RH_FACTOR:
			pNode->bf = EH_FACTOR;
			if(pNode->tree_root && pNode->bf == EH_FACTOR)
			{
				if(pNode->tree_root->left_child == pNode)
					avlDelBalance(pTree , pNode->tree_root , LEFT_MINUS);
				else
					avlDelBalance(pTree , pNode->tree_root , RIGHT_MINUS);
			}
			break;
		}
	}

	return 1;
}


/******************************************************************** 
* 
* AVL_TREE_LOCK(tAVLTree *pTree , int timeout)
* 
* 锁定二叉树，防止多个任务同时对树进行添加或删除操作
* 此函数是针对vxworks系统的扩展，如果不是vxworks系统，那么树的互斥操作
* 需要自定义
* timeout		: 等待时间，vxworks操作系统里面timeout=1就是1/60秒
*
* Returns         :  无
*********************************************************************/ 
void AVL_TREE_LOCK
(
 tAVLTree *pTree,
 int timeout
 )
{
	if(!pTree
#if OS==3 || OS==4		
		|| !pTree->sem
#endif		
		)
		return;

#if OS==3 || OS==4
	semTake(pTree->sem,timeout);
#endif
	return;
}

/********************************************************************* 
* 
* AVL_TREE_UNLOCK(tAVLTree *pTree , int timeout)
* 
* 解除锁定
* 此函数是针对vxworks系统的扩展，如果不是vxworks系统，那么树的互斥操作
* 需要自定义
* Returns         :  无
*********************************************************************/ 
void AVL_TREE_UNLOCK
(
 tAVLTree *pTree
 )
{
	if(!pTree
#if OS==3 || OS==4		
		|| !pTree->sem
#endif		
		)
		return;

#if OS==3 || OS==4
	semGive(pTree->sem);
#endif
	return;
}

/******************************************************************** 
* 
* AVL_TREENODE_FREE(tAVLTree *pTree , TREE_NODE *pNode)
* 
* 释放一个节点所占用的内存，释放函数需要用户自定义
* ，并且需要在创建二叉树的时候传递给二叉树
* 
* Returns         :  无
*********************************************************************/ 
void AVL_TREENODE_FREE
(
 tAVLTree *pTree,
 TREE_NODE *pNode
 )
{
	if(!pTree || !pNode)
		return;

	(*pTree->free)(pNode);
	return ;
}

#ifdef ORDER_LIST_WANTED
/******************************************************************************** 
* 
* orderListInsert
*	(
*	tAVLTree *pTree,	      //树结构的指针	
*	TREE_NODE *pNode ,    //pInsertNode即将插在此节点前面或后面
*	TREE_NODE *pInsertNode, //即将插入的节点指针
*	int prev_or_next      // INSERT_PREV : 待插入节点插在pNode之前
*                                           INSERT_NEXT : 待插入节点插在pNode之后           
*	)
* 
*   当平衡二叉树里增加一个节点之后，用此函数来更新
*  有序双向链表
* 
* Returns         :  1:成功  0:失败
*********************************************************************************/ 
static int orderListInsert
(
 tAVLTree *pTree,
 TREE_NODE *pNode , 
 TREE_NODE *pInsertNode,
 int prev_or_next
 )
{
	TREE_NODE *p = AVL_NULL;

	if(!pNode)
		return 0;

	if(prev_or_next == INSERT_PREV)
	{
		p = pNode->prev;
		if(p)	p->next = pInsertNode;
		else	pTree->pListHeader = pInsertNode;

		pInsertNode->prev = p;
		pInsertNode->next = pNode;
		pNode->prev = pInsertNode;
	}

	if(prev_or_next == INSERT_NEXT)
	{
		p = pNode->next;
		if(p)	p->prev = pInsertNode;
		else	pTree->pListTail = pInsertNode;

		pInsertNode->prev = pNode;
		pInsertNode->next = p;
		pNode->next = pInsertNode;
	}
	return 1;
}

/******************************************************************** 
* int orderListRemove
*	(
*	tAVLTree *pTree,    //树结构的指针
*	TREE_NODE *pRemoveNode   //即将从有序双向链表中删除的节点
*	)
* 
*   当平衡二叉树里删除一个节点之后，用此函数来更新
*  有序双向链表
* 
* Returns         :  1:成功   0:失败
********************************************************************/ 
static int orderListRemove
(
 tAVLTree *pTree,
 TREE_NODE *pRemoveNode
 )
{
	TREE_NODE *pPrev = AVL_NULL;
	TREE_NODE *pNext = AVL_NULL;

	if(!pRemoveNode)
		return 0;

	pPrev = pRemoveNode->prev;
	pNext = pRemoveNode->next;
	if(!pPrev && !pNext)
	{
		pTree->pListHeader = pTree->pListTail = AVL_NULL;
		return 1;
	}
	if(pPrev && pNext)
	{
		pPrev->next = pNext;
		pNext->prev = pPrev;
		return 1;
	}

	if(pPrev)
	{
		pPrev->next = AVL_NULL;
		pTree->pListTail = pPrev;
		return 1;
	}

	if(pNext)
	{
		pNext->prev = AVL_NULL;
		pTree->pListHeader = pNext;
		return 1;
	}
	else 
	{
		return 0;
	}
}


/******************************************************************** 
*      avlTreeFirst(tAVLTree *pTree)
* 
*   获取有序双向链表里面的第一个成员节点
* 
* Returns         :  成功:  第一个成员节点的指针
*                         失败:  AVL_NULL
*********************************************************************/ 
TREE_NODE *avlTreeFirst
(
 tAVLTree *pTree
 )
{
	if(!pTree)
		return AVL_NULL;

	if(!pTree->count || !pTree->pTreeHeader)
		return AVL_NULL;

	return (TREE_NODE *)pTree->pListHeader;
}


/******************************************************************** 
*      avlTreeLast(tAVLTree *pTree)
* 
*   获取有序双向链表里面的最后一个成员节点
* 
* Returns         :  成功:  最后一个成员节点的指针
*                         失败:  AVL_NULL
*********************************************************************/ 
TREE_NODE *avlTreeLast
(
 tAVLTree *pTree
 )
{
	if(!pTree)
		return AVL_NULL;

	if(!pTree->count || !pTree->pTreeHeader)
		return AVL_NULL;

	return (TREE_NODE *)pTree->pListTail;
}

/******************************************************************** 
*      avlTreeNext(TREE_NODE *pNode)
* 
*   获取有序双向链表里面当前成员节点的后一个节点
* 
* Returns         :  成功: 后一个成员节点的指针
*                         失败:  AVL_NULL
*********************************************************************/ 
TREE_NODE *avlTreeNext
(
 TREE_NODE *pNode
 )
{
	if(!pNode)
		return AVL_NULL;

	return (TREE_NODE *)pNode->next;
}

/******************************************************************** 
*      avlTreePrev(TREE_NODE *pNode)
* 
*   获取有序双向链表里面当前成员节点的前一个节点
* 
* Returns         :  成功: 前一个成员节点的指针
*                         失败:  AVL_NULL
*********************************************************************/ 
TREE_NODE *avlTreePrev
(
 TREE_NODE *pNode
 )
{
	if(!pNode)
		return AVL_NULL;

	return (TREE_NODE *)pNode->prev;
}
#endif

/*****************************************************************************************
*      int avlTreeInsert
*	(
*	tAVLTree *pTree ,      //树结构的指针
*	TREE_NODE **ppNode ,  //待插入节点所在的子树的指针的指针
*	TREE_NODE *pInsertNode,  //待插入的节点
*	int *growthFlag  //子树是否长高的标志 *growthFlag=1表示长高1层 *growthFlag=0表示没有
*	)
* 
*   将一个节点插入一颗子树之中，插入过程之中可能导致子树不
*  平衡，此函数还将执行递归平衡操作，直到所有子树均平衡为止
* 
* Returns         :  1:成功
*                         0:失败
******************************************************************************************/ 
static int avlTreeInsert
(
 tAVLTree *pTree , 
 TREE_NODE **ppNode , 
 TREE_NODE *pInsertNode,
 int *growthFlag
 )
{
	int compFlag = 0;
	TREE_NODE *pNode = (TREE_NODE *)(*ppNode);

	if(pTree->count == 0)
	{
		pTree->pTreeHeader = pInsertNode;
		pInsertNode->bf = EH_FACTOR;
		pInsertNode->left_child = pInsertNode->right_child = AVL_NULL;
		pInsertNode->tree_root = AVL_NULL;
#ifdef ORDER_LIST_WANTED
		pTree->pListHeader = pTree->pListTail = pInsertNode;
		pInsertNode->prev = pInsertNode->next = AVL_NULL;
#endif
		return 1;
	}

	compFlag = ((*pTree->keyCompare)(pNode , pInsertNode));
	if(!compFlag)
	{
		*growthFlag = 0;
		return 0;
	}

	if(compFlag < 0)
	{
		if(!pNode->left_child)
		{
			pNode->left_child = pInsertNode;
			pInsertNode->bf = EH_FACTOR;
			pInsertNode->left_child = pInsertNode->right_child = AVL_NULL;
			pInsertNode->tree_root = (TREE_NODE *)pNode;
#ifdef ORDER_LIST_WANTED
			orderListInsert(pTree,pNode, pInsertNode, INSERT_PREV);
#endif
			switch(pNode->bf)
			{
			case EH_FACTOR:
				pNode->bf = LH_FACTOR;
				*growthFlag = 1;
				break;
			case RH_FACTOR:
				pNode->bf = EH_FACTOR;
				*growthFlag = 0;
				break;
			}
		}
		else
		{
			if(!avlTreeInsert(pTree, &pNode->left_child,pInsertNode, growthFlag))
				return 0;

			if(*growthFlag)
			{
				switch(pNode->bf)
				{
				case LH_FACTOR:
					LeftBalance(ppNode);
					*growthFlag = 0;
					break;
				case EH_FACTOR:
					pNode->bf = LH_FACTOR;
					*growthFlag = 1;
					break;
				case RH_FACTOR:
					pNode->bf = EH_FACTOR;
					*growthFlag = 0;
					break;
				}
			}
		}
	}

	if(compFlag > 0)
	{
		if(!pNode->right_child)
		{
			pNode->right_child = pInsertNode;
			pInsertNode->bf = EH_FACTOR;
			pInsertNode->left_child = pInsertNode->right_child = AVL_NULL;
			pInsertNode->tree_root = (TREE_NODE *)pNode;
#ifdef ORDER_LIST_WANTED
			orderListInsert(pTree,pNode, pInsertNode, INSERT_NEXT);
#endif
			switch(pNode->bf)
			{
			case EH_FACTOR:
				pNode->bf = RH_FACTOR;
				*growthFlag = 1;
				break;
			case LH_FACTOR:
				pNode->bf = EH_FACTOR;
				*growthFlag = 0;
				break;
			}
		}
		else
		{
			if(!avlTreeInsert(pTree, &pNode->right_child,pInsertNode, growthFlag))
				return 0;

			if(*growthFlag)
			{
				switch(pNode->bf)
				{
				case LH_FACTOR:
					pNode->bf = EH_FACTOR;
					*growthFlag = 0;
					break;
				case EH_FACTOR:
					pNode->bf = RH_FACTOR;
					*growthFlag = 1;
					break;
				case RH_FACTOR:
					RightBalance(ppNode);
					*growthFlag = 0;
					break;
				}
			}
		}
	}

	return 1;
}


/******************************************************************** 
*      int avlTreeRemove
*	(
*	tAVLTree *pTree ,      //树结构的指针
*	TREE_NODE *pRemoveNode  //待删除节点的指针
*	)
* 
*   从树里面删除一个节点，此函数能够做递归操作，能够
*  循环自平衡，使所有受删除节点影响而导致不平衡的子树
*  都能自平衡
* 
* Returns         :  1:成功
*                    0:失败
*                                                    
*          C               C                                                           
*         / \             / \                     C                                     
*        B   E    ==>    B  .F.      ==>         / \                                     
*       /   / \         /   / \                 B  .F.                                   
*      A   D   G       A   D   G               /   / \                                   
*             / \             / \             A   D   G                                 
*            F   H          .E.  H                     \                                  
*                                                       H                  
*      删除E节点  ==> 找到比E大一点的F ==>  删除E节点，自平衡                                                           
*                     F和E互换指针                                                
********************************************************************/ 
static int avlTreeRemove
(
 tAVLTree *pTree , 
 TREE_NODE *pRemoveNode
 )
{
	int compFlag = 0;
	TREE_NODE *tree_root = AVL_NULL;
	TREE_NODE *p = AVL_NULL;
	TREE_NODE *root_p = AVL_NULL;
	TREE_NODE swapNode;

	tree_root = pRemoveNode->tree_root;
	if(!pRemoveNode->left_child && !pRemoveNode->right_child)
	{
		if(!tree_root)
		{
			pTree->pTreeHeader = AVL_NULL;
#ifdef ORDER_LIST_WANTED
			pTree->pListHeader = pTree->pListTail = AVL_NULL;
#endif
			return 1;
		}
		else if(tree_root->left_child == pRemoveNode)
		{
#ifdef ORDER_LIST_WANTED
			orderListRemove(pTree, pRemoveNode);
#endif
			tree_root->left_child = AVL_NULL;
			avlDelBalance(pTree, tree_root , LEFT_MINUS);
		}
		else
		{
#ifdef ORDER_LIST_WANTED
			orderListRemove(pTree, pRemoveNode);
#endif
			tree_root->right_child = AVL_NULL;
			avlDelBalance(pTree, tree_root , RIGHT_MINUS);
		}
	}

	if(pRemoveNode->left_child && pRemoveNode->right_child)
	{
		TREE_NODE *prev = AVL_NULL;
		TREE_NODE *next = AVL_NULL;
		TREE_NODE *r_child = AVL_NULL;
		root_p = pRemoveNode;
		p = pRemoveNode->right_child;
		while(p->left_child)
		{
			root_p = p;
			p = p->left_child;
		}
		if(p == pRemoveNode->right_child)
		{
			p->tree_root = p;
			pRemoveNode->right_child = pRemoveNode;
		}
		swapNode = *p;
		prev = p->prev;
		next = p->next;
		*p = *pRemoveNode;
		p->prev = prev;
		p->next = next;
		prev = pRemoveNode->prev;
		next = pRemoveNode->next;
		*pRemoveNode = swapNode;
		pRemoveNode->prev = prev;
		pRemoveNode->next = next;
		if(!tree_root) 
			pTree->pTreeHeader = p;
		else if(tree_root->left_child == pRemoveNode)
			tree_root->left_child = p;
		else
			tree_root->right_child = p;

		if(p->left_child) 
			p->left_child->tree_root = p;
		if(p->right_child)  
			p->right_child->tree_root = p;

		if(pRemoveNode->left_child) 
			pRemoveNode->left_child->tree_root = pRemoveNode;
		if(pRemoveNode->right_child)  
			pRemoveNode->right_child->tree_root = pRemoveNode;

		if(root_p != pRemoveNode)
		{
			if(root_p->left_child == p)
				root_p->left_child = pRemoveNode;
			else 
				root_p->right_child = pRemoveNode;
		}

		return avlTreeRemove(pTree, pRemoveNode);
	}

	if(pRemoveNode->left_child)
	{
#ifdef ORDER_LIST_WANTED
		orderListRemove(pTree, pRemoveNode);
#endif
		if(!tree_root)
		{
			pTree->pTreeHeader = pRemoveNode->left_child;
			pRemoveNode->left_child->tree_root = AVL_NULL;
			return 1;
		}

		if(tree_root->left_child == pRemoveNode)
		{
			tree_root->left_child = pRemoveNode->left_child;
			pRemoveNode->left_child->tree_root= tree_root;
			avlDelBalance(pTree , tree_root , LEFT_MINUS);
		}
		else
		{
			tree_root->right_child = pRemoveNode->left_child;
			pRemoveNode->left_child->tree_root = tree_root;
			avlDelBalance(pTree , tree_root , RIGHT_MINUS);
		}

		return 1;
	}

	if(pRemoveNode->right_child)
	{
#ifdef ORDER_LIST_WANTED
		orderListRemove(pTree, pRemoveNode);
#endif
		if(!tree_root)
		{
			pTree->pTreeHeader = pRemoveNode->right_child;
			pRemoveNode->right_child->tree_root = AVL_NULL;
			return 1;
		}

		if(tree_root->left_child == pRemoveNode)
		{
			tree_root->left_child = pRemoveNode->right_child;
			pRemoveNode->right_child->tree_root = tree_root;
			avlDelBalance(pTree , tree_root , LEFT_MINUS);
		}
		else
		{
			tree_root->right_child = pRemoveNode->right_child;
			pRemoveNode->right_child->tree_root = tree_root;
			avlDelBalance(pTree , tree_root , RIGHT_MINUS);
		}

		return 1;
	}

	return 1;
}

/******************************************************************** 
*      int avlTreeLookup
*	(
*	tAVLTree *pTree,
*	TREE_NODE *pNode , 
*	TREE_NODE *pSearchKey
*	)
* 
*    递归查找关键字比较完全匹配的节点，比较函数是在
*     树创建的时候就指定好的
*
* Returns         :  1:成功
*                         0:失败
*********************************************************************/ 
static TREE_NODE *avlTreeLookup
(
 tAVLTree *pTree,
 TREE_NODE *pNode , 
 TREE_NODE *pSearchKey
 )
{
	int compFlag = 0;
	if(!pTree || !pNode)
		return AVL_NULL;

	compFlag = (*pTree->keyCompare)(pNode , pSearchKey);
	if(!compFlag)
		return (TREE_NODE *)pNode;

	if(compFlag>0) pNode = pNode->right_child;
	else pNode = pNode->left_child;

	return (TREE_NODE *)avlTreeLookup(pTree, pNode, pSearchKey);
}


/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            : 创建一颗有序平衡二叉树
★参数描述: 
keyCompareFunc:比较两个节点的大小(关键字的比较)
★返回值      :
成功 :   平衡二叉树的指针
失败 :   空指针
*******************************************************************/
tAVLTree *avlTreeCreate(int *keyCompareFunc,int *freeFunc)
{
	tAVLTree *pTree = (tAVLTree *)0;

	if(!keyCompareFunc || !freeFunc)
		return (tAVLTree *)0;

	pTree = (tAVLTree *)malloc(sizeof(tAVLTree));
	
	if(pTree != (tAVLTree *)0)
	{
		memset((void *)pTree , 0 , sizeof(tAVLTree));
		pTree->keyCompare = (void *)keyCompareFunc;
		pTree->free = (void *)freeFunc;
#ifdef ORDER_LIST_WANTED
		pTree->pListHeader = pTree->pListTail = AVL_NULL;
#endif

#if OS==3 || OS==4 
		pTree->sem = semBCreate(0 , 1);
		if(!pTree->sem)
		{
			free((void *)pTree);
			return (tAVLTree *)0;
		}
#endif
	}

	return (tAVLTree *)pTree;
}

/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            :  删除一个节点

★参数描述: 
pTree:树结构的指针
pDelNode : 待删除的节点指针
★返回值      :
成功 :  1
失败 :   0
*******************************************************************/
int avlTreeDel( tAVLTree *pTree ,TREE_NODE *pDelNode)
{
	int ret = 0;

	if(!pTree || !pDelNode || !pTree->count)
		return 0;

	ret = avlTreeRemove(pTree, pDelNode);
	if(ret)
		pTree->count--;

	return 1;
}


/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            : 摧毁一颗平衡二叉树，并释放所有成员节点占用的内存
释放内存的函数在创建树的时候已经指定好
★参数描述: 
pTree:树结构的指针
★返回值      :
成功 :  1
失败 :   0
********************************************************************/
int avlTreeDestroy
(
 tAVLTree *pTree
 )
{
	TREE_NODE *pNode = AVL_NULL;
	if(!pTree)
		return 0;

	while(pNode = pTree->pTreeHeader)
	{
		avlTreeDel(pTree,pNode);
		AVL_TREENODE_FREE(pTree, pNode);
	}

	if(!pTree->count || !pTree->pTreeHeader)
	{
#if OS==3 || OS==4
		semDelete(pTree->sem);
#endif
		free((void *)pTree);
		return 1;
	}

	return 0;
}


/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            : 清空一颗树，释放所有成员节点占用的内存，
但是不释放树结构所占用的内存
★参数描述: 
pTree:树结构的指针
★返回值      :
成功 :  1
失败 :   0
********************************************************************/
int avlTreeFlush
(
 tAVLTree *pTree
 )
{
	TREE_NODE *pNode = AVL_NULL;

	if(!pTree)
		return 0;

	if(!pTree->count || !pTree->pTreeHeader)
		return 1;

	while(pNode = pTree->pTreeHeader)
	{
		avlTreeDel(pTree,pNode);
		AVL_TREENODE_FREE(pTree, pNode);
	}

	return 0;
}


/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            :  增加一个节点

★参数描述: 
pTree:树结构的指针
pInsertNode : 待添加的节点指针
★返回值      :
成功 :  1
失败 :   0
*******************************************************************/
int avlTreeAdd
(
 tAVLTree *pTree , 
 TREE_NODE *pInsertNode
 )
{
	int growthFlag=0 , ret = 0;

	if(!pTree || !pInsertNode)
		return 0;

	ret = avlTreeInsert(pTree , &pTree->pTreeHeader , pInsertNode , &growthFlag);
	if(ret)
		pTree->count++;
	return ret;
}



/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            : 根据关键字结构来查询一个节点是否存在

★参数描述: 
pTree:树结构的指针
pKeyNode : 关键字结构指针
★返回值      :
成功 :  查找到的节点指针
失败 :   AVL_NULL
********************************************************************/
TREE_NODE *avlTreeFind
(
 tAVLTree *pTree,
 TREE_NODE *pKeyNode
 )
{
	if(!pTree || !pTree->count || !pTree->pTreeHeader)
		return AVL_NULL;

	return (TREE_NODE *)avlTreeLookup(pTree, pTree->pTreeHeader , pKeyNode);
}

/*******************************************************************/
/**************************AVL TREE API*****************************/
/*******************************************************************/
/*
★描述            : 获取树里面的所有节点总数

★参数描述: 
pTree:树结构的指针
★返回值      :
树里面的节点成员总数
********************************************************************/
unsigned int avlTreeCount
(
 tAVLTree *pTree
 )
{
	if(!pTree)
		return 0;

	return pTree->count;
}


