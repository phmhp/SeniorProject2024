package com.example.roverosong

import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.google.firebase.database.*

class GuideActivity : AppCompatActivity() {

    private lateinit var database: DatabaseReference

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.guide)

        val guideText = findViewById<TextView>(R.id.guideText)
        val subText = findViewById<TextView>(R.id.guideSubText)

        // Firebase Realtime Database의 status 참조
        database = FirebaseDatabase.getInstance().getReference("UserResults/status")

        // Firebase 데이터 변경 감지 리스너 추가
        database.addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                val status = snapshot.getValue(String::class.java)

                if (status == "종료") {
                    // "종료" 상태일 때 텍스트 업데이트
                    guideText.text = "안내가 종료되었습니다."
                    subText.text = "이용해주셔서 감사합니다."
                } else if (status == "준비") {
                    // "준비" 상태가 되면 MainActivity로 자동 이동
                    navigateToMainActivity()
                }
            }

            override fun onCancelled(error: DatabaseError) {
                Log.e("FirebaseError", "데이터 읽기 실패: ${error.message}")
            }
        })

        // 전체화면 모드 설정 (네비게이션 바 숨김)
        window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                )
    }

    private fun navigateToMainActivity() {
        val intent = Intent(this@GuideActivity, MainActivity::class.java)
        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_NEW_TASK)
        startActivity(intent)
        finish() // 현재 액티비티 종료
    }
}
