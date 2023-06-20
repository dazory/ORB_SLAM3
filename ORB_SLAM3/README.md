# ORB-SLAM3

## Week3

If (IMU measured && enough acceleration): New map created with 513 points.

1. `Tracking::Track()`
  
    If we have an initial estimation of the camera pose and matching. Track the local map.
    
    1. `Tracking::TrackLocalMap()`

        1. UpdateLocalMap

            1. `mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);`
            2. UpdateLocalKeyFrames: `mvpLocalMapPoints`에 관련있는 keyframes을 모두 push.
            3. UpdateLocalPoints: `mvpLocalMapPoints`에 관련있는 map points를 모두 push.

        2. SearchLocalPoints

            1. 현재 프레임의 모든 `mvpMapPoints`에 대해, (1) increaseVisible() (2) set  `LastFrameSeen=mCurrentFrame.mnId; mbTrackInView=false; mbTrackInViewR=false;`
                * `mbTrackInView=false`: 아래에서 `mmProjectPoints`에 projection하지 않게됨.

            2. 현재 프레임의 모든 `mvpLocalMapPoints`에 대해:

                * Frustum 내에 존재하는 map point이면: increaseVisible() && `nToMatch++`
                * `mbTrackInView=true`: `mCurrentFrame.mmProjectPoints`에 `(mTrackProjX, mTrackProjY)`을 할당.

            3. match해야하는 map points가 존재하면 (`nToMatch>0`)

                * ORBmatcher(`nnratio=0.8`)를 생성하고, `mvpLocalMapPoints`을 `th`으로 `mCurrentFrame`에 projection.

        * If (TrackLocalMap success): `bOK=true`

    2. Save frame & Reset: `mState = OK;`

    3. (If `bOK=true`) Update model & Clean, Delete

        1. Update motion model => `mVelocity`

        2. Clean VO matches: For all map points in current frame (pMP), set mvbOutliers=false & mvpMapPoints=NULL if `observation < 1`.

        3. Delete temporal MapPoints.

        4. (If `bOK=true`) NeedNewKeyFrame()

            * (If `IMU` && `IMU.initialized` && timestemp interval <= 0.25): return false // => Not CreateNewKeyFrame; (초반에 여기를 3번정도 돎)
            * (If `IMU` && `IMU.initialized` && timestemp interval >= 0.25): return true // => CreateNewKeyFrame; (이후에 여기로 들어옴)

    4. (If `NeedNewKeyFrame` && bOK) CreateNewKeyFrame()

        * (If `Stereo` && ) Update pose matrices: `mCurrentFrame.UpdatePoseMatrices();`

            * `vector<pair<float,int>> vDepthIdx.push_back(z, i)` for `z`: mCurrentFrame.mvDepth; `i`: index of keypoints;
            * (If `!vDepthIdx.empty`) For all `vDepthIdx` (`j`): 

                For all index of mvDepth (`i`):
                1. (If mvpMapPoints[i].empty) mvpMapPoints[i] = NULL; `bCreateNew=true`
                2. (If `bCreateNew`) 
                    1. x3D = mCurrentFrame.UnprojectStereo(i)
                    2. pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap())
                    3. pNewMP->AddObservation(pKF, i); pKF->AddMapPoint(pNewMP, i);
                    4. pNewMP-ComputeDistictiveDescriptors(); pNewMP->UpdateNormalAndDepth();
                    5. mpAtlas->AddMapPoint(pNewMP)
                    6. mCurrentFrame.mvpMapPoints[i]=pNewMP
                3. nPoints++
                4. (If `z>mThDepth` && `nPoints>maxPoint`) break;
        * mpLocalMapper->InsertKeyFrame(pKF)
        * mpLocalMapper->SetNotStop(false)
        * mnLastKeyFrameId = mCurrentFrame.mnId
        * mpLastKeyFrame = pKF
    
    6. (If `mState=LOST`) Reset when LOST
        
        * (If `pCurrentMap->KeyFramesInMap()<=5` or `IMU && !IMU.initialized`) mpSystem->ResetActiveMap(); return;
        * CreateMapInAtlas()

    7.  mLastFrame = Frame(mCurrentFrame)
    8. (If `mState==OK` or `mState==RECENTLY_LOST`) Save frame pose information

        * (If `!mCurrentFrame.mTcw.empty()`) store mpReferenceKF's information
            * `mlRelativeFramePoses` <- `mCurrentFrame.mpReferenceKF->GetPoseInverse()`
            * `mlpReferences` <- `mCurrentFrame.mpReferenceKF`
            * `mlFrameTimes` <- `mCurrentFrame.mTimeStamp`
            * `mlbLost` <- `mState==LOST`
        * (else: If tracking is lost) store privious mpReferenceKF's information"
            * `mlRelativeFramePoses` <- `mlRelativeFramePoses.back()`
            * `mlpReferences` <- `mlpReferences.back()`
            * `mlFrameTimes` <- `mlFrameTimes.back()`
            * `mlbLost` <- `mState==LOST`


2. `LocalMapping::Run()`

    Repeat below:

    1. Tracking will see that Local Mapping is busy: `mbAcceptKeyFrames=false`;

    2. (If `CheckNewKeyFrames()` && `!mbBadImu`)

        1. BoW conversion and insertion in Map: `ProcessNewKeyFrame()`

            1. Compute Bags of Words structures: ComputeBoW()

                : `mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4)`
            2. For all vpMapPointMatches(pMP),
                
                * AddObservation(mpCurrentKeyFrame, i): `mbObservations`에 `mpCurrentKeyFrame을 저장
                * UpdateNormalAndDepth(): keyframes로부터 normal과 depth를 계산하여 `mNormalVector`와 `mfMinDistance`&`mfMaxDistance`에 저장.

                    * For all mObservations (pKF, indexes), compute `normal` using mWorldPos and camera pose.
                    * Get level from Keyframes.octave
                    * Compute `mfMinDistance` and `mfMaxDistance` using dist and level.
                    * Compute `mNormalVector`=normal / num_observations.

                * ComputeDistinctiveDescriptors(): 

                    * For all mObservations (pKF, indexes), Push back pKF->mDescriptors into `vDescriptors`.
                    * For all vDescriptors, Compute `Distances`.
                    * Take the descriptors with least median distance to the reset: `mDescriptor=vDescriptors[BestIdx].clone()`.
            3. Update links in the Covisibility graph: `mpCurrentKeyFrame->UpdateConnections()`.

                * For all mvpMapPoints(pMP), KFcounter++ as much it has good observations.
                * For all KFcounter(KF, index), 
                    * Update `nmax`: the max of index.
                    * Update `pKFmax`: the keyframe with max index.
                    * Push back (KF, index) into `vPairs` and Add connection between them if index >= th.
                * For all vPairs(KF, index), lKFs.push_fron(index) & lWs.push_front(KF).
                * Update member variables:
                    * `mConnectedKeyFrameWeights`=KFCounter
                    * `mvpOrderedConnectedKeyFrames`=lKFs
                    * `mvOrderedWeights`=lWs
                    * If first connection, `mpParent`=mvpOrderedConnectedKeyFrames.front() & add child(this) & `mbFirstConnection`=false.

            4. Insert Keyframe in Map: `mpAtlas->AddKeyFrame(mpCurrentKeyFrame)`.

        2. Check recent MapPoints: `MapPointCulling()`

        3. Triangulate new MapPoints: `CreateNewMapPoints()`

        4. (If `!CheckNewKeyFrames()`) Find more matches in neighbor keyframes and fuse point duplications: `SearchInNeighbors()`; `num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA = 0`

        5. (If `!CheckNewKeyFrames()` && `!stopRequested()`)

            * (If `mpAtlas->KeyFramesInMap()>2`) BundleAdjustment; `b_doneLBA=true`
                * mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
            * (If `b_doneLBA`) push back
                * `vnLBA_edges` <- `num_edges_BA`
                * `vnLBA_KFopt` <- `num_OptKF_BA`
                * `vnLBA_KFfixed` <- `num_FixedKF_BA`
                * `vnLBA_MPs` <- `num_MPs_BA`
            * (If `!IMU.initialized` && `mbInertial`) InitializeImu

            * Check redundant local keyframes: `KeyFrameCulling()`

            * (If `mTinit<100.0f` && `mbInertial` && `IMU.initialized` && `Tracking::OK`) 

                * Get InertialBA and initialize IMU
                * (If `mpAtlas->KeyFramesInMap())<=100` && `mTinit...`) ScaleRefinement()
3. 