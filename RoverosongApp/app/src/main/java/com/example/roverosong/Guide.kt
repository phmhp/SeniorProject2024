package com.example.roverosong

import android.os.Bundle
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

        // Firebase Realtime Database 참조
        database = FirebaseDatabase.getInstance().getReference("UserResults/status")

        // Firebase 데이터 변경 감지
        database.addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                val status = snapshot.getValue(String::class.java)
                if (status == "종료") {
                    // 데이터가 "종료" 상태로 변경되면 텍스트 업데이트
                    guideText.text = "안내가 종료되었습니다."
                    subText.text = "이용해주셔서 감사합니다."
                }
            }

            override fun onCancelled(error: DatabaseError) {
                // 오류 처리 (필요 시 로그 출력)
                println("Database error: ${error.message}")
            }
        })

        // 전체화면 모드 설정
        window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                )
    }
}
