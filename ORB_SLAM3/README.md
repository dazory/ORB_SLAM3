# ORB-SLAM3

## Week3

If (IMU measured && enough acceleration): New map created with 513 points.

1. `Tracking::Track()`
  
    If we have an initial estimation of the camera pose and matching. Track the local map.
    
    1. `Tracking::TrackLocalMap()`

        1. UpdateLocalMap => 목적: 빨간 map points를 보고 있는 local map을 형성

            1. `mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);`
            2. UpdateLocalKeyFrames: `mvpLocalMapPoints`에 관련있는 keyframes을 모두 push.
                * `mvpLocalKeyFrames`<- keyframes 중 current frame가 보는 map points을 똑같이 보고있는 keyframes -- (a)
                * `mvpLocalKeyFrames`<- `mvpLocalKeyFrames`와 covisibility 공유하는 frames
                * `mvpLocalKeyFrames` <- (a)가 parent일 때 childs들을 set으로(겹치는건 하나만), childs: covisibility graph에서 어케 구하는거임.
                * `mvpLocalKeyFrames` <- (a)가 child일 때 parent들도 가져옴
                * `mvpLocalKeyFrames` <- 최근 20 frames도 넣음
                * `mpReferenceKF` <- observations이 가장 많이 겹치는 애.

            3. UpdateLocalPoints: `mvpLocalMapPoints`에 관련있는 map points를 모두 push.

        2. SearchLocalPoints: currentFrame이 못찾은 map points를 추가

            1. 현재 프레임의 모든 `mvpMapPoints`에 대해, (1) increaseVisible() (2) set  `LastFrameSeen=mCurrentFrame.mnId; mbTrackInView=false; mbTrackInViewR=false;`
                * `mbTrackInView=false`: 아래에서 `mmProjectPoints`에 projection하지 않게됨.

            2. 현재 프레임의 모든 `mvpLocalMapPoints`에 대해:

                * Frustum 내에 존재하는 map point이면: increaseVisible() && `nToMatch++`: currentFrame에서 못봤는데 그래도 Frustum 내에 있으면 본 걸로 치겠다.
                * `mbTrackInView=true`: `mCurrentFrame.mmProjectPoints`에 `(mTrackProjX, mTrackProjY)`을 할당.

            3. match해야하는 map points가 존재하면 (`nToMatch>0`): match 개수 구하고 끝남.

                * ORBmatcher(`nnratio=0.8`)를 생성하고, `mvpLocalMapPoints`을 `th`(radius), recent면 신뢰도 높으니까 th를 크게함,으로 `mCurrentFrame`에 projection.
                
                => currentFrame(F)의 mvpMapPoints에 넣어줌..

        * If (TrackLocalMap success): `bOK=true`

    2. Save frame & Reset: `mState = OK;`

    3. (If `bOK=true`) Update model & Clean, Delete

        1. Update motion model => `mVelocity`

        2. Clean VO matches: For all map points in current frame (pMP), set mvbOutliers=false & mvpMapPoints=NULL if `observation < 1`.

        3. Delete temporal MapPoints.

        4. (If `bOK=true`) NeedNewKeyFrame()

            * (If `IMU` && `IMU.initialized` && timestemp interval <= 0.25): return false // => Not CreateNewKeyFrame; (초반에 여기를 3번정도 돎) // IMU쓰면 1초에 4번 keyframe 무조건 보장됨.
            * (If `IMU` && `IMU.initialized` && timestemp interval >= 0.25): return true // => CreateNewKeyFrame; (이후에 여기로 들어옴)

            * `nTrackedClose`: currentF의 mapPoints와 너무 멀리 떨어진 애들은 부정확하다 생각.
                * `nTrackedClose`< 100: (inlier) 너무 겹치면 keyframe으로서 의미 없으니까 100 미만이어야함 (depth 미만인 애들. 이건 위치 변하면 슉슉 변하는 애들. 그래서 얘네가 좀 유의미함. )
                * `nNonTrackedCLose`>70: outlier.
                => 슬슬 내가 어디 보는지 모르겠다. 그러니까 얠 keyframe으로 지정하겠다.
            * 여튼 위 것들이 conditions으로 엄청 많은데.. 그걸로 keyframe 할지(true) 말지(false) 결정함. 

    4. (If `NeedNewKeyFrame` && bOK) CreateNewKeyFrame()

        * (If `Stereo` && ) Update pose matrices: `mCurrentFrame.UpdatePoseMatrices();`

            * `vector<pair<float,int>> vDepthIdx.push_back(z, i)` for `z`: mCurrentFrame.mvDepth; `i`: index of keypoints;
            * (If `!vDepthIdx.empty`) For all `vDepthIdx` (`j`): 

                For all index of mvDepth (`i`):
                1. (If mvpMapPoints[i].empty) mvpMapPoints[i] = NULL; `bCreateNew=true`
                2. (If `bCreateNew`)  // 지금 이게 새로 만들어진 map을 atlas에 추가한거임
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
    
    5. (If `mState=LOST`) Reset when LOST
        
        * (If `pCurrentMap->KeyFramesInMap()<=5` or `IMU && !IMU.initialized`) mpSystem->ResetActiveMap(); return;
        * CreateMapInAtlas()

    6.  mLastFrame = Frame(mCurrentFrame)
    7. (If `mState==OK` or `mState==RECENTLY_LOST`) Save frame pose information

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